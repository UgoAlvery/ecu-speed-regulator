#ifndef ECU_TASK_RX_H
#define ECU_TASK_RX_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "protocol.h"
#include "board_pins.h"

/* Protocole sur UART2 (broches dans board_pins.h) : UART0 reste réservé à la
 * console ESP_LOG / idf.py monitor, qui ne peut donc plus s'insérer au milieu
 * d'une trame (ENF-03). task_tx écrit sur ce même port. */
#define UART_NUM        UART_NUM_2
#define UART_BUF_SIZE   1024
#define UART_BAUD_RATE  115200
#define RX_QUEUE_SIZE   16

void task_rx_init(QueueHandle_t rx_queue, QueueHandle_t mode_queue);
void task_rx(void *pvParameters);

uint32_t task_rx_get_count_valid(void);
uint32_t task_rx_get_count_crc_err(void);
uint32_t task_rx_get_count_dropped(void);
uint32_t task_rx_get_count_replay(void);
uint32_t task_rx_get_count_mode_invalid(void);
uint32_t task_rx_get_count_mode_overwritten(void);

#endif /* ECU_TASK_RX_H */