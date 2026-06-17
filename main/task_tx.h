//
// Created by ugoal on 29/05/2026.
//

#ifndef ECU_TASK_TX_H
#define ECU_TASK_TX_H

#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "protocol.h"

#define TX_QUEUE_SIZE   8

typedef struct {
    uint8_t  type;
    uint8_t  payload[PROTOCOL_MAX_PAYLOAD_SIZE];
    uint16_t payload_len;
} tx_message_t;

void     task_tx_init(QueueHandle_t tx_queue);
void     task_tx(void *pvParameters);
uint32_t task_tx_get_count_output(void);

#endif //ECU_TASK_TX_H