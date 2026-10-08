//
// Created by ugoal on 29/05/2026.
//

#include "task_tx.h"
#include "protocol.h"
#include "task_rx.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

static const char *TAG = "task_tx";

#define TASK_TX_UART_NUM    UART_NUM   /* UART2 protocole, installé par task_rx_init */

#define TASK_TX_WRITE_TIMEOUT_MS  20

/* Attente bornée sur queue_tx (au lieu de portMAX_DELAY) pour que task_tx
 * puisse nourrir le TWDT même sans trafic sortant — doit rester nettement
 * inférieur à CONFIG_ESP_TASK_WDT_TIMEOUT_S (2 s). */
#define TASK_TX_IDLE_WAKE_MS      500

static QueueHandle_t s_tx_queue = NULL;

static volatile uint32_t s_count_tx_output = 0;

/* Nonce sortant : task_tx est le seul écrivain UART et le seul encodeur —
 * pas de mutex nécessaire (même exception structurelle que les compteurs
 * volatile de task_rx). Démarre à 1 : protocol_nonce_check_and_update()
 * rejette un nonce égal à la valeur initiale du contexte récepteur (0). */
static uint16_t s_tx_nonce = 0;


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

bool task_tx_emit_output(const float output, void *ctx)
{
    const QueueHandle_t queue = (QueueHandle_t)ctx;
    if (queue == NULL) {
        return false;
    }

    /* task_tx est le SEUL encodeur : on lui transmet le payload BRUT. */
    tx_message_t msg;
    msg.type        = MSG_OUTPUT;
    memcpy(msg.payload, &output, sizeof(float));
    msg.payload_len = (uint16_t)sizeof(float);

    /* FIN de queue, jamais en tête : l'ordre des OUTPUT dans queue_tx doit
     * rester celui des commits sous mutex (rta_fmea.md §6.1 bis). Timeout 0 :
     * appelé mutex_ecu_state tenu, ne doit jamais bloquer. */
    return xQueueSendToBack(queue, &msg, 0) == pdTRUE;
}

void task_tx(void *pvParameters)
{
    (void)pvParameters;

    configASSERT(s_tx_queue != NULL);

    uint8_t   frame_buf[PROTOCOL_MAX_FRAME_SIZE];
    tx_message_t msg;

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    ESP_LOGI(TAG, "tâche démarrée (prio %d)", uxTaskPriorityGet(NULL));

    for (;;) {
        esp_task_wdt_reset();

        if (xQueueReceive(s_tx_queue, &msg,
                          pdMS_TO_TICKS(TASK_TX_IDLE_WAKE_MS)) != pdTRUE) {
            continue;
        }

        if (msg.payload_len > PROTOCOL_MAX_PAYLOAD_SIZE) {
            ESP_LOGW(TAG, "payload_len=%u dépasse MAX — trame ignorée",
                     msg.payload_len);
            continue;
        }

        const size_t frame_len = protocol_encode(frame_buf,
                                           msg.type,
                                           ++s_tx_nonce,
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