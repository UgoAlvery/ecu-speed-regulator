#include "task_control.h"

#include "ecu_state.h"
#include "pid.h"
#include "protocol.h"
#include "task_timing.h"
#include "task_tx.h"

#if ECU_USE_ENCODER_SPEED
#include "hal_encoder.h"
#endif

#if ECU_DRIVE_HW_PWM
#include "hal_pwm.h"
#endif

#if ECU_USE_ENCODER_SPEED || ECU_DRIVE_HW_PWM
#include "hal_config.h"
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_task_wdt.h"


#define TAG                      "CTRL"
#define CONTROL_PERIOD_MS        100u
/* Defense-in-depth: disable PWM if no valid RX frame received for this long
 * while in AUTO mode. Must be < FAILSAFE_TIMEOUT_MS (2000 ms). */
/* Primary heartbeat guard runs in task_failsafe (ctrl_heartbeat.h): fires even
 * if this task blocks. This secondary guard fires if the task runs but receives
 * no valid frame. Must be < FAILSAFE_TIMEOUT_MS (2000 ms). */
#define CTRL_SILENCE_GUARD_MS    1500u

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

/* SETPOINT / SPEED bien formés mais hors bornes de plausibilité
 * (PROTOCOL_SETPOINT_* / PROTOCOL_SPEED_*) : rejetés, dernière valeur valide
 * conservée. Écrivain unique task_control. */
static volatile uint32_t s_count_rx_out_of_range = 0;

/* MODE_SET(AUTO) refusés parce que la ligne failsafe GPIO est ouverte
 * (verrou posé par task_failsafe, cf. ecu_state_try_set_mode). */
static volatile uint32_t s_count_mode_refused = 0;

/* Incremented at the top of every loop iteration; monitored by task_failsafe.
 * volatile: written by task_control, read by task_failsafe (different tasks). */
static volatile uint32_t s_control_heartbeat = 0;

static void apply_pending_mode(void);
static void dispatch_pending_frames(void);
static void dispatch_frame(const ecu_frame_t *frame);
static float compute_requested_output(pid_t *pid, ecu_mode_t mode);
static void  emit_output(float requested);


void task_control_init(const QueueHandle_t rx_queue, const QueueHandle_t mode_queue,
                       const QueueHandle_t tx_queue)
{
    configASSERT(rx_queue != NULL);
    configASSERT(mode_queue != NULL);
    configASSERT(tx_queue != NULL);
    s_rx_queue   = rx_queue;
    s_mode_queue = mode_queue;
    s_tx_queue   = tx_queue;
#if ECU_USE_ENCODER_SPEED
    configASSERT(hal_encoder_init());
#endif
#if ECU_DRIVE_HW_PWM
    configASSERT(hal_pwm_init());
    configASSERT(hal_pwm_enable()); /* requires -DHWPROTECTION_CONFIRMED=true */
#endif
}

uint32_t task_control_get_count_tx_dropped(void) { return s_count_tx_dropped; }
uint32_t task_control_get_count_rx_invalid(void) { return s_count_rx_invalid; }
uint32_t task_control_get_count_mode_refused(void) { return s_count_mode_refused; }
uint32_t task_control_get_count_rx_out_of_range(void) { return s_count_rx_out_of_range; }
uint32_t task_control_get_heartbeat(void) { return s_control_heartbeat; }

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
        s_control_heartbeat++;
        esp_task_wdt_reset();
        timing_record_control_tick();
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


static void count_rejection(const protocol_value_status_t status)
{
    if (status == PROTOCOL_VALUE_OUT_OF_RANGE) {
        s_count_rx_out_of_range++;
    } else {
        s_count_rx_invalid++;
    }
}


static void dispatch_frame(const ecu_frame_t *frame)
{
    /* Validation de longueur, de domaine et de plausibilité dans protocol.c
     * (lib pure, testée en host). Valeur rejetée : la dernière valeur valide
     * est conservée. */
    float val;
    protocol_value_status_t status;

    switch (frame->type) {

    case MSG_SETPOINT:
        status = protocol_parse_f32_bounded(frame, PROTOCOL_SETPOINT_MIN,
                                            PROTOCOL_SETPOINT_MAX, &val);
        if (status == PROTOCOL_VALUE_OK) {
            ecu_state_set_setpoint(val);
        } else {
            count_rejection(status);
        }
        break;

    case MSG_SPEED:
        status = protocol_parse_f32_bounded(frame, PROTOCOL_SPEED_MIN,
                                            PROTOCOL_SPEED_MAX, &val);
        if (status == PROTOCOL_VALUE_OK) {
            ecu_state_set_speed(val);
        } else {
            count_rejection(status);
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
#if ECU_USE_ENCODER_SPEED
    const float speed = hal_encoder_get_speed_kmh(CONTROL_PERIOD_MS / 1000.0f);
#else
    const float speed = ecu_state_get_speed();
#endif
    return pid_compute(pid, setpoint, speed);
}


static void emit_output(const float requested)
{
    /* Relecture du mode, choix de la commande ET mise en queue (fin de file,
     * timeout 0) sous une seule prise du mutex. task_failsafe passe en OFF et
     * poste OUTPUT(0) sous ce même mutex : une commande non nulle est donc
     * soit en queue AVANT son OUTPUT(0), soit remplacée par 0 ici. Aucune
     * trame non nulle ne peut suivre l'OUTPUT(0) du failsafe (rta_fmea.md
     * §6.1 bis). */
    if (!ecu_state_commit_output(requested, task_tx_emit_output, s_tx_queue)) {
        /* Pas de log : ce chemin ne s'exécute qu'en surcharge (rta_fmea.md
         * §4.3). Le compteur tx_dropped remonte via STATS. */
        s_count_tx_dropped++;
    }
#if ECU_DRIVE_HW_PWM
    /* Defense-in-depth: disable PWM if no valid frame received for 1.5 s
     * while in AUTO mode (task_failsafe fires at 2 s — this is earlier).
     * Uses uint32 subtraction which wraps correctly for windows << 49 days. */
    {
        const TickType_t now     = xTaskGetTickCount();
        const TickType_t last_rx = ecu_state_get_last_rx_tick();
        const bool silence = (ecu_state_get_mode() == ECU_MODE_AUTO) &&
                             ((TickType_t)(now - last_rx) >
                              pdMS_TO_TICKS(CTRL_SILENCE_GUARD_MS));
        if (silence) {
            hal_pwm_disable();
        } else {
            /* Map PID output [0, PID_DEFAULT_OUT_MAX] to duty [0, 10000] */
            const float norm = requested / PID_DEFAULT_OUT_MAX;
            const uint16_t duty = (norm > 1.0f) ? 10000u
                                : (norm < 0.0f) ? 0u
                                : (uint16_t)(norm * 10000.0f);
            hal_pwm_set_duty_pct(duty);
        }
    }
#endif
}