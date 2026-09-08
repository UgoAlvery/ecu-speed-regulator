//
// Created by ugoal on 29/05/2026.
//

#ifndef ECU_TASK_TELEMETRY_H
#define ECU_TASK_TELEMETRY_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define TELEMETRY_STATS_COUNT   6U
#define TELEMETRY_PERIOD_MS     1000U

void task_telemetry_init(QueueHandle_t tx_queue);
void task_telemetry(void *pvParameters);

#endif //ECU_TASK_TELEMETRY_H