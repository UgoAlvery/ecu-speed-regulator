#ifndef FAILSAFE_H
#define FAILSAFE_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define FAILSAFE_GPIO_PIN      GPIO_NUM_4

#define FAILSAFE_TIMEOUT_MS    2000

#define FAILSAFE_RESPONSE_MS   5


void failsafe_init(QueueHandle_t tx_queue);


void task_failsafe(void *pvParameters);

#endif /* FAILSAFE_H */