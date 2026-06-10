//
// Created by ugoal on 29/05/2026.
//

#ifndef ECU_TASK_CONTROL_H
#define ECU_TASK_CONTROL_H

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void task_control_init(QueueHandle_t rx_queue, QueueHandle_t tx_queue);
void task_control(void *pvParameters);

#endif //ECU_TASK_CONTROL_H