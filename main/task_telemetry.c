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

static volatile uint32_t s_tx_output_count = 0U;

void task_telemetry_init(const QueueHandle_t tx_queue)
{
    configASSERT(tx_queue != NULL);
    s_tx_queue = tx_queue;
}

void task_telemetry_inc_tx_output(void)
{
    s_tx_output_count++;
}

void task_telemetry(void *pvParameters)
{
    (void)pvParameters;

    configASSERT(s_tx_queue != NULL);
    static uint8_t  s_payload[TELEMETRY_STATS_COUNT * sizeof(uint32_t)];
    static uint8_t  s_frame[PROTOCOL_MAX_FRAME_SIZE];

    TickType_t      last_wake = xTaskGetTickCount();
    uint32_t        uptime_s  = 0U;

    for (;;)
    {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
        uptime_s++;

        uint32_t rx_valid     = task_rx_get_count_valid();
        uint32_t rx_crc_error = task_rx_get_count_crc_err();
        uint32_t rx_dropped   = task_rx_get_count_dropped();

        uint32_t tx_output = s_tx_output_count;

        uint8_t  *p = s_payload;

        memcpy(p, &rx_valid,     sizeof(uint32_t)); p += sizeof(uint32_t);
        memcpy(p, &rx_crc_error, sizeof(uint32_t)); p += sizeof(uint32_t);
        memcpy(p, &rx_dropped,   sizeof(uint32_t)); p += sizeof(uint32_t);
        memcpy(p, &tx_output,    sizeof(uint32_t)); p += sizeof(uint32_t);
        memcpy(p, &uptime_s,     sizeof(uint32_t));

        const uint16_t payload_len = (uint16_t)(TELEMETRY_STATS_COUNT * sizeof(uint32_t));


        const size_t frame_len = protocol_encode(s_frame, MSG_STATS,
                                           s_payload, payload_len);
        if (frame_len == 0U) {
            continue;
        }

        tx_message_t msg;
        memcpy(msg.payload, s_frame, frame_len);
        msg.payload_len = (uint16_t)frame_len;

        xQueueSend(s_tx_queue, &msg, 0);
    }
    vTaskDelete(NULL);
}