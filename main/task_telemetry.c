//
// Created by ugoal on 29/05/2026.
//

#include "task_telemetry.h"


#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "protocol.h"
#include "ecu_state.h"
#include "task_rx.h"
#include "task_tx.h"


static QueueHandle_t s_tx_queue = NULL;

void task_telemetry_init(const QueueHandle_t tx_queue)
{
    configASSERT(tx_queue != NULL);
    s_tx_queue = tx_queue;
}

void task_telemetry(void *pvParameters)
{
    (void)pvParameters;

    configASSERT(s_tx_queue != NULL);
    static uint8_t s_payload[TELEMETRY_STATS_COUNT * sizeof(uint32_t)];

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t   uptime_s  = 0U;

    for (;;)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
        uptime_s++;

        uint32_t rx_valid     = task_rx_get_count_valid();
        uint32_t rx_crc_error = task_rx_get_count_crc_err();
        uint32_t rx_dropped   = task_rx_get_count_dropped();
        uint32_t tx_output    = task_tx_get_count_output();

        uint8_t *p = s_payload;
        memcpy(p, &rx_valid,     sizeof(uint32_t)); p += sizeof(uint32_t);
        memcpy(p, &rx_crc_error, sizeof(uint32_t)); p += sizeof(uint32_t);
        memcpy(p, &rx_dropped,   sizeof(uint32_t)); p += sizeof(uint32_t);
        memcpy(p, &tx_output,    sizeof(uint32_t)); p += sizeof(uint32_t);
        memcpy(p, &uptime_s,     sizeof(uint32_t));

        const uint16_t payload_len =
            (uint16_t)(TELEMETRY_STATS_COUNT * sizeof(uint32_t));

        /* task_tx est le SEUL encodeur : on lui transmet le payload BRUT. */
        tx_message_t msg;
        msg.type        = MSG_STATS;
        memcpy(msg.payload, s_payload, payload_len);
        msg.payload_len = payload_len;

        xQueueSend(s_tx_queue, &msg, 0);
    }
}