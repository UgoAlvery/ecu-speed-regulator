#include "failsafe.h"

#include <string.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "ecu_state.h"
#include "protocol.h"


static SemaphoreHandle_t s_sem_failsafe = NULL;
static QueueHandle_t     s_tx_queue     = NULL;


static void send_alarm(const char *cause)
{
    static const size_t MAX_ALARM_LEN = 64;

    uint8_t frame_buf[PROTOCOL_MAX_FRAME_SIZE];
    uint8_t payload[64];

    size_t cause_len = strlen(cause);
    if (cause_len > MAX_ALARM_LEN) {
        cause_len = MAX_ALARM_LEN;
    }
    memcpy(payload, cause, cause_len);

    const size_t frame_len = protocol_encode(frame_buf, MSG_ALARM, payload,
                                       (uint16_t)cause_len);


    typedef struct {
        uint8_t data[PROTOCOL_MAX_FRAME_SIZE];
        size_t  len;
    } tx_message_t;

    tx_message_t msg;
    memcpy(msg.data, frame_buf, frame_len);
    msg.len = frame_len;
    xQueueSend(s_tx_queue, &msg, 0);
}


static void trigger_failsafe(const char *cause)
{
    ecu_state_set_output(0.0f);
    ecu_state_set_mode(ECU_MODE_OFF);

    send_alarm(cause);
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    (void)arg;

    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_sem_failsafe, &higher_priority_task_woken);


    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void failsafe_init(QueueHandle_t tx_queue)
{
    s_tx_queue = tx_queue;

    s_sem_failsafe = xSemaphoreCreateBinary();
    configASSERT(s_sem_failsafe != NULL);

    const gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FAILSAFE_GPIO_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    ESP_ERROR_CHECK(gpio_isr_handler_add(FAILSAFE_GPIO_PIN,
                                         gpio_isr_handler, NULL));
}


void task_failsafe( void *pvParameters)
{
    (void)pvParameters;

    const TickType_t polling_period = pdMS_TO_TICKS(100);
    const TickType_t timeout_ticks  = pdMS_TO_TICKS(FAILSAFE_TIMEOUT_MS);

    for (;;) {
        const BaseType_t gpio_event = xSemaphoreTake(s_sem_failsafe, polling_period);

        if (gpio_event == pdTRUE) {
            trigger_failsafe("FAILSAFE: GPIO error signal detected");
        } else {
            const TickType_t now       = xTaskGetTickCount();
            const TickType_t last_tick = ecu_state_get_last_rx_tick();
            if ((now - last_tick) >= timeout_ticks) {
                const ecu_mode_t current_mode = ecu_state_get_mode();
                if (current_mode != ECU_MODE_OFF) {
                    trigger_failsafe("FAILSAFE: UART silence > 2s");
                }
            }
        }
    }
}