#include "failsafe.h"

#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "ecu_state.h"
#include "failsafe_line.h"
#include "protocol.h"
#include "task_tx.h"


/* Ligne failsafe en contact normalement fermé (NC) : jumper GPIO4 → GND,
 * pull-up interne. Repos = 0, ouverture (jumper retiré OU fil coupé) = 1.
 * Contrairement à un contact NO, la rupture du câblage est elle-même une
 * demande de failsafe — plus de défaut latent (rta_fmea.md F-13). */
#define LINE_OPEN_LEVEL          1
#define POLLING_PERIOD_MS        100u

static SemaphoreHandle_t s_sem_failsafe = NULL;
static QueueHandle_t     s_tx_queue     = NULL;

/* État local à task_failsafe (seul écrivain, seul lecteur). Le latch GPIO
 * et l'anti-rebond de reprise sont dans failsafe_line.c (lib pure, testée
 * en host) ; le déclenchement n'attend aucune stabilisation (EF-05). */
static failsafe_line_t s_line;
static bool            s_uart_latched = false;  /* failsafe silence UART actif */

/* ALARM non mises en queue faute de place (queue_tx pleine, 8 messages).
 * Écrivain unique task_failsafe, lu par task_telemetry (STATS). */
static volatile uint32_t s_count_alarm_dropped = 0;

/* OUTPUT(0) du failsafe non mis en queue (queue_tx pleine). Additionné par
 * task_telemetry au compteur de task_control dans tx_dropped : deux
 * compteurs à écrivain unique plutôt qu'un compteur à deux écrivains. */
static volatile uint32_t s_count_output_dropped = 0;


static void send_alarm(const char *cause)
{
    /* task_tx est le SEUL encodeur : on lui transmet le payload BRUT. */
    tx_message_t msg;

    size_t cause_len = strlen(cause);
    if (cause_len > PROTOCOL_MAX_PAYLOAD_SIZE) {
        cause_len = PROTOCOL_MAX_PAYLOAD_SIZE;
    }

    msg.type        = MSG_ALARM;
    memcpy(msg.payload, cause, cause_len);
    msg.payload_len = (uint16_t)cause_len;

    /* En TÊTE de queue_tx : l'ALARM passe devant les OUTPUT/STATS en attente
     * et n'attend que la trame en cours d'écriture (rta_fmea.md §6.1).
     * Timeout 0 : échoue seulement si queue_tx est pleine → compteur. */
    if (xQueueSendToFront(s_tx_queue, &msg, 0) != pdTRUE) {
        s_count_alarm_dropped++;
    }
}


static void update_led(void)
{
    gpio_set_level((gpio_num_t)FAILSAFE_LED_PIN,
                   (s_line.latched || s_uart_latched) ? 1 : 0);
}


/* Verrou (si lock), OFF, commande 0 ET OUTPUT(0) en fin de queue_tx, sous une
 * seule prise du mutex (D10) : l'arrêt est annoncé immédiatement, et aucune
 * commande non nulle ne peut suivre cet OUTPUT(0). Si queue_tx est pleine,
 * task_control émet OUTPUT(0) à son prochain cycle (≤ 100 ms). L'ALARM part
 * ensuite en tête de queue. lock : verrou anti-AUTO (ligne GPIO). */
static void trigger_failsafe(const char *cause, const bool lock)
{
    if (!ecu_state_enter_failsafe(lock, task_tx_emit_output, s_tx_queue)) {
        s_count_output_dropped++;
    }
    send_alarm(cause);
}


/* Exécute l'action décidée par failsafe_line. TRIP n'est retourné qu'à la
 * transition vers l'état failsafe : rebonds et niveaux à 1 répétés tombent
 * sur le latch et n'émettent aucune nouvelle ALARM. */
static void apply_line_action(const failsafe_line_action_t action, const char *cause)
{
    switch (action) {
    case FAILSAFE_LINE_TRIP:
        /* Verrou, commande 0 et OFF sous une seule prise du mutex : un
         * MODE_SET(AUTO) concurrent est soit appliqué avant (puis écrasé par
         * OFF), soit refusé. */
        trigger_failsafe(cause, true);
        update_led();
        break;
    case FAILSAFE_LINE_UNLOCK:
        ecu_state_set_failsafe_locked(false);   /* reprise EF-06 de nouveau possible */
        update_led();
        break;
    case FAILSAFE_LINE_NONE:
    default:
        break;
    }
}


static bool line_is_open(void)
{
    return gpio_get_level((gpio_num_t)FAILSAFE_GPIO_PIN) == LINE_OPEN_LEVEL;
}


static void poll_uart_silence(const TickType_t timeout_ticks)
{
    const ecu_mode_t current_mode = ecu_state_get_mode();

    /* Fin du failsafe UART : reprise par MODE_SET valide (EF-06). */
    if (s_uart_latched && current_mode != ECU_MODE_OFF) {
        s_uart_latched = false;
        update_led();
    }

    const TickType_t now       = xTaskGetTickCount();
    const TickType_t last_tick = ecu_state_get_last_rx_tick();
    if ((now - last_tick) >= timeout_ticks && current_mode != ECU_MODE_OFF) {
        s_uart_latched = true;
        trigger_failsafe("FAILSAFE: UART silence > 2s", false);
        update_led();
    }
}


static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    (void)arg;

    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_sem_failsafe, &higher_priority_task_woken);


    portYIELD_FROM_ISR(higher_priority_task_woken);
}

uint32_t failsafe_get_count_alarm_dropped(void) { return s_count_alarm_dropped; }
uint32_t failsafe_get_count_output_dropped(void) { return s_count_output_dropped; }

void failsafe_init(QueueHandle_t tx_queue)
{
    s_tx_queue = tx_queue;
    failsafe_line_init(&s_line);

    s_sem_failsafe = xSemaphoreCreateBinary();
    configASSERT(s_sem_failsafe != NULL);

    /* Front montant = ouverture du contact NC. */
    const gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FAILSAFE_GPIO_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    const gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << FAILSAFE_LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&led_conf));
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)FAILSAFE_LED_PIN, 0));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)FAILSAFE_GPIO_PIN,
                                         gpio_isr_handler, NULL));
}


void task_failsafe( void *pvParameters)
{
    (void)pvParameters;

    const TickType_t polling_period = pdMS_TO_TICKS(POLLING_PERIOD_MS);
    const TickType_t timeout_ticks  = pdMS_TO_TICKS(FAILSAFE_TIMEOUT_MS);

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    /* Ligne déjà ouverte au démarrage (fil coupé avant le boot) : aucun front
     * ne viendra jamais, seul le niveau le révèle. Les trames partent dès que
     * task_tx tourne (queue_tx créée avant les tâches). */
    if (line_is_open()) {
        apply_line_action(failsafe_line_on_level(&s_line, true),
                          "FAILSAFE: GPIO line open at boot");
    }

    TickType_t last_poll = xTaskGetTickCount();

    for (;;) {
        esp_task_wdt_reset();

        /* Attente bornée par la prochaine échéance de polling : une rafale de
         * fronts ne décale pas le polling (période fixe de 100 ms). */
        const TickType_t elapsed = xTaskGetTickCount() - last_poll;
        const TickType_t wait    = (elapsed < polling_period)
                                 ? (polling_period - elapsed) : 0;
        const BaseType_t gpio_event = xSemaphoreTake(s_sem_failsafe, wait);

        if (gpio_event == pdTRUE) {
            /* Premier front : déclenchement immédiat, sans attendre que le
             * niveau soit stable. Les rebonds suivants retombent sur le
             * latch et n'émettent rien. */
            apply_line_action(failsafe_line_on_edge(&s_line),
                              "FAILSAFE: GPIO line opened (edge)");
        }

        const TickType_t now = xTaskGetTickCount();
        if ((now - last_poll) >= polling_period) {
            last_poll = now;
            /* Niveau relu à chaque poll : couvre un front manqué et gère
             * le déverrouillage quand la ligne est refermée. */
            apply_line_action(failsafe_line_on_level(&s_line, line_is_open()),
                              "FAILSAFE: GPIO line open (level)");
            poll_uart_silence(timeout_ticks);
        }
    }
}
