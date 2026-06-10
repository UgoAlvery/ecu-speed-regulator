//
// Created by ugoal on 29/05/2026.
//

#include "task_tx.h"
#include "protocol.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "task_tx";

#define TASK_TX_UART_NUM    UART_NUM_0

#define TASK_TX_WRITE_TIMEOUT_MS  20

static QueueHandle_t s_tx_queue = NULL;

static volatile uint32_t s_count_tx_output = 0;


void task_tx_init(const QueueHandle_t tx_queue)
{
    configASSERT(tx_queue != NULL);
    s_tx_queue = tx_queue;
    ESP_LOGI(TAG, "init OK — queue_tx handle enregistré");
}

uint32_t task_tx_get_count_output(void)
{
    return s_count_tx_output;
}

void task_tx(void *pvParameters)
{
    (void)pvParameters;

    configASSERT(s_tx_queue != NULL);

    uint8_t   frame_buf[PROTOCOL_MAX_FRAME_SIZE];
    tx_message_t msg;

    ESP_LOGI(TAG, "tâche démarrée (prio %d)", uxTaskPriorityGet(NULL));

    for (;;) {

        if (xQueueReceive(s_tx_queue, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (msg.payload_len > PROTOCOL_MAX_PAYLOAD_SIZE) {
            ESP_LOGW(TAG, "payload_len=%u dépasse MAX — trame ignorée",
                     msg.payload_len);
            continue;
        }

        const size_t frame_len = protocol_encode(frame_buf,
                                           msg.type,
                                           msg.payload,
                                           msg.payload_len);
        if (frame_len == 0) {
            ESP_LOGE(TAG, "protocol_encode() a échoué pour type=0x%02X", msg.type);
            continue;
        }

        const int written = uart_write_bytes(TASK_TX_UART_NUM,
                                       (const char *)frame_buf,
                                       frame_len);
        if (written < 0) {
            ESP_LOGE(TAG, "uart_write_bytes() erreur pour type=0x%02X", msg.type);
            continue;
        }

        if ((size_t)written != frame_len) {

            ESP_LOGW(TAG, "écriture partielle : %d/%u octets pour type=0x%02X",
                     written, (unsigned)frame_len, msg.type);
        }

        if (msg.type == MSG_OUTPUT) {
            s_count_tx_output++;
        }

        ESP_LOGD(TAG, "TX type=0x%02X len=%u", msg.type, (unsigned)frame_len);
    }

    vTaskDelete(NULL);
}