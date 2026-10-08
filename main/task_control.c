#include "task_control.h"

#include <string.h>

#include "ecu_state.h"
#include "pid.h"
#include "protocol.h"
#include "task_tx.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_task_wdt.h"


#define TAG               "CTRL"
#define CONTROL_PERIOD_MS 100u

static QueueHandle_t s_rx_queue   = NULL;
static QueueHandle_t s_mode_queue = NULL;
static QueueHandle_t s_tx_queue = NULL;

/* task_control est le seul écrivain de ce compteur (même exception structurelle
 * que les compteurs volatile de task_rx) — nombre d'OUTPUT non émis faute de
 * place dans queue_tx. Trouvé et corrigé lors de la RTA formelle (cf.
 * rta_fmea.md) : enqueue_output() bloquait jusqu'à 10 ms sur xQueueSend en cas
 * de queue pleine, un terme de blocage non borné dans le budget de jitter
 * ±5 ms de task_control. */
static volatile uint32_t s_count_tx_dropped = 0;

/* Trames saines au niveau protocole (CRC + nonce OK) mais rejetées au
 * dispatch : longueur de payload différente de celle du type, valeur non
 * finie / hors domaine, ou type non attendu en réception. Remplace les
 * ESP_LOG* du dispatch : un log par trame rejetée est déclenchable par
 * l'émetteur et coûte du temps dans le cycle 100 ms (rta_fmea.md §4.3).
 * Même écrivain unique (task_control). */
static volatile uint32_t s_count_rx_invalid = 0;

/* MODE_SET(AUTO) refusés parce que la ligne failsafe GPIO est ouverte
 * (verrou posé par task_failsafe, cf. ecu_state_try_set_mode). */
static volatile uint32_t s_count_mode_refused = 0;

static void apply_pending_mode(void);
static void dispatch_pending_frames(void);
static void dispatch_frame(const ecu_frame_t *frame);
static float compute_requested_output(pid_t *pid, ecu_mode_t mode);
static void  emit_output(float requested);
static bool  enqueue_output(float output);


void task_control_init(const QueueHandle_t rx_queue, const QueueHandle_t mode_queue,
                       const QueueHandle_t tx_queue)
{
    configASSERT(rx_queue != NULL);
    configASSERT(mode_queue != NULL);
    configASSERT(tx_queue != NULL);
    s_rx_queue   = rx_queue;
    s_mode_queue = mode_queue;
    s_tx_queue   = tx_queue;
}

uint32_t task_control_get_count_tx_dropped(void) { return s_count_tx_dropped; }
uint32_t task_control_get_count_rx_invalid(void) { return s_count_rx_invalid; }
uint32_t task_control_get_count_mode_refused(void) { return s_count_mode_refused; }

void task_control(void *pvParameters)
{
    (void)pvParameters;

    configASSERT(s_rx_queue != NULL);
    configASSERT(s_tx_queue != NULL);

    pid_t pid;
    pid_init(&pid,
             PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD,
             PID_DEFAULT_DT, PID_DEFAULT_OUT_MIN, PID_DEFAULT_OUT_MAX);

    ecu_mode_t prev_mode = ecu_state_get_mode();

    TickType_t last_wake = xTaskGetTickCount();

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    ESP_LOGI(TAG, "task_control démarrée, période %u ms", CONTROL_PERIOD_MS);

    for (;;) {
        esp_task_wdt_reset();
        apply_pending_mode();       /* avant queue_frames : un OFF n'attend rien */
        dispatch_pending_frames();

        const ecu_mode_t mode = ecu_state_get_mode();
        if (mode != prev_mode) {
            pid_reset(&pid);
            prev_mode = mode;
        }

        /* OUTPUT émis à CHAQUE cycle, dans tous les modes (0 hors AUTO) :
         * task_control est l'unique émetteur d'OUTPUT (R-9). Une trame perdue
         * est remplacée en ≤ 100 ms, et l'OUTPUT sert de signal de vie à
         * l'actionneur (coupure si silence > 300 ms côté ECU moteur). */
        emit_output(compute_requested_output(&pid, mode));

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * Fonctions internes
 * ══════════════════════════════════════════════════════════════════════════ */

/* MODE_SET arrive par queue_mode (longueur 1, écrasée par task_rx), jamais
 * par queue_frames : lu sans bloquer en tout début de cycle. La transition
 * (PID reset, commande à 0 hors AUTO) est ensuite faite par la boucle, sur
 * changement de ecu_state_get_mode(). */
static void apply_pending_mode(void)
{
    ecu_mode_t mode;
    if (xQueueReceive(s_mode_queue, &mode, 0) == pdTRUE) {
        if (!ecu_state_try_set_mode(mode)) {
            s_count_mode_refused++;
        }
    }
}


static void dispatch_pending_frames(void)
{
    ecu_frame_t frame;
    while (xQueueReceive(s_rx_queue, &frame, 0) == pdTRUE) {
        dispatch_frame(&frame);
    }
}


static void dispatch_frame(const ecu_frame_t *frame)
{
    /* Validation de longueur et de domaine dans protocol.c (lib pure, testée
     * en host). Valeur rejetée : la dernière valeur valide est conservée. */
    float val;

    switch (frame->type) {

    case MSG_SETPOINT:
        if (protocol_parse_f32(frame, &val)) {
            ecu_state_set_setpoint(val);
        } else {
            s_count_rx_invalid++;
        }
        break;

    case MSG_SPEED:
        if (protocol_parse_f32(frame, &val)) {
            ecu_state_set_speed(val);
        } else {
            s_count_rx_invalid++;
        }
        break;

    default:
        s_count_rx_invalid++;
        break;
    }
}


static float compute_requested_output(pid_t *pid, const ecu_mode_t mode)
{
    if (mode != ECU_MODE_AUTO) {
        return 0.0f;
    }
    const float setpoint = ecu_state_get_setpoint();
    const float speed    = ecu_state_get_speed();
    return pid_compute(pid, setpoint, speed);
}


static void emit_output(const float requested)
{
    /* Relecture du mode juste avant la mise en queue, atomique avec
     * l'écriture de la commande : si task_failsafe est passé entre le calcul
     * du PID et ici, la valeur retenue est 0. Seul reste possible un failsafe
     * APRÈS ce point : au plus une trame non nulle déjà validée part après
     * l'ALARM, remplacée par OUTPUT(0) au cycle suivant (rta_fmea.md F-20). */
    const float output = ecu_state_commit_output(requested);

    if (!enqueue_output(output)) {
        /* Pas de log : ce chemin ne s'exécute qu'en surcharge (rta_fmea.md
         * §4.3). Le compteur tx_dropped remonte via STATS. */
        s_count_tx_dropped++;
    }
}


static bool enqueue_output(const float output)
{
    /* task_tx est le SEUL encodeur : on lui transmet le payload BRUT. */
    tx_message_t msg;
    msg.type        = MSG_OUTPUT;
    memcpy(msg.payload, &output, sizeof(float));
    msg.payload_len = (uint16_t)sizeof(float);

    /* Timeout 0 (non bloquant), comme partout ailleurs dans l'architecture
     * (xQueueSend task_rx/failsafe/telemetry, xQueueReceive task_control) :
     * task_control ne doit jamais attendre sur une ressource externe pendant
     * son cycle 100 ms — rejet silencieux + compteur, pas de blocage. */
    return xQueueSend(s_tx_queue, &msg, 0) == pdTRUE;
}