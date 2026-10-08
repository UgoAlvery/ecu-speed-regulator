//
// Created by ugoal on 29/05/2026.
//

#ifndef ECU_TASK_CONTROL_H
#define ECU_TASK_CONTROL_H

#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void task_control_init(QueueHandle_t rx_queue, QueueHandle_t mode_queue,
                       QueueHandle_t tx_queue);
void task_control(void *pvParameters);
uint32_t task_control_get_count_tx_dropped(void);
uint32_t task_control_get_count_rx_invalid(void);
uint32_t task_control_get_count_mode_refused(void);
uint32_t task_control_get_count_rx_out_of_range(void);

#endif //ECU_TASK_CONTROL_H