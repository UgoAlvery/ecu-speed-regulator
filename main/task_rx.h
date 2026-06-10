#ifndef ECU_TASK_RX_H
#define ECU_TASK_RX_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "protocol.h"

#define UART_NUM        UART_NUM_0
#define UART_BUF_SIZE   1024
#define UART_BAUD_RATE  115200
#define TXD_PIN         17
#define RXD_PIN         16
#define RX_QUEUE_SIZE   16

void task_rx_init(QueueHandle_t rx_queue);
void task_rx(void *pvParameters);

uint32_t task_rx_get_count_valid(void);
uint32_t task_rx_get_count_crc_err(void);
uint32_t task_rx_get_count_dropped(void);

#endif /* ECU_TASK_RX_H */