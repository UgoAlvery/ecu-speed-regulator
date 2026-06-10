#include "task_rx.h"
#include "protocol.h"
#include <string.h>

#include "driver/uart.h"

#define PARSER_TIMEOUT_MS 50
#define FRAME_TIMEOUT_MS 2000

typedef enum {
    STATE_WAIT_START,
    STATE_READ_LEN,
    STATE_READ_PAYLOAD,
    STATE_READ_CRC,
} parser_state_t;

typedef struct {
    parser_state_t state;
    uint8_t buf[PROTOCOL_MAX_PAYLOAD_SIZE];
    size_t buf_idx;
    uint16_t len_field;
    uint8_t len_bytes_read;
} frame_parser_t;

static QueueHandle_t s_rx_queue;
static frame_parser_t s_parser;

static volatile uint32_t s_count_valid   = 0;
static volatile uint32_t s_count_crc_err = 0;
static volatile uint32_t s_count_dropped = 0;

static void uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &cfg);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void parser_reset(frame_parser_t *p)
{
    p->state          = STATE_WAIT_START;
    p->buf_idx        = 0;
    p->len_field      = 0;
    p->len_bytes_read = 0;
}

static bool parser_feed_byte(frame_parser_t *p, const uint8_t byte, ecu_frame_t *out) {
    switch (p->state) {

        case STATE_WAIT_START:
            if (byte == PROTOCOL_START_BYTE) {
                parser_reset(p);
                p->buf[p->buf_idx++] = byte;
                p->state = STATE_READ_LEN;
            }

            break;

        case STATE_READ_LEN:
            p->buf[p->buf_idx++] = byte;
            p->len_bytes_read++;

            if (p->len_bytes_read == 2) {
                p->len_field = (uint16_t)p->buf[1] | ((uint16_t)p->buf[2] << 8);

                if (p->len_field < 1 ||
                    p->len_field > (PROTOCOL_MAX_PAYLOAD_SIZE + 1)) {
                    parser_reset(p);
                    s_count_dropped++;
                    break;
                    }
                p->state = STATE_READ_PAYLOAD;
            }
            break;

        case STATE_READ_PAYLOAD:
            p->buf[p->buf_idx++] = byte;

            if (p->buf_idx == 1 + 2 + p->len_field) {
                p->state = STATE_READ_CRC;
            }
            break;

        case STATE_READ_CRC:
            p->buf[p->buf_idx++] = byte;

            if (protocol_decode(p->buf, p->buf_idx, out)) {
                parser_reset(p);
                return true;
            } else {
                s_count_crc_err++;
                parser_reset(p);
                return false;
            }

        default:
            parser_reset(p);
            break;
    }

    return false;
}

// API

void task_rx_init(const QueueHandle_t rx_queue)
{
    s_rx_queue = rx_queue;
    parser_reset(&s_parser);
    uart_init();
}

uint32_t task_rx_get_count_valid(void)   { return s_count_valid;   }
uint32_t task_rx_get_count_crc_err(void) { return s_count_crc_err; }
uint32_t task_rx_get_count_dropped(void) { return s_count_dropped; }

void task_rx(void *pvParameters) {
    (void)pvParameters;
    uint8_t byte;
    ecu_frame_t frame;

    while (1) {
        const int received = uart_read_bytes(UART_NUM, &byte, 1,
                                       pdMS_TO_TICKS(PARSER_TIMEOUT_MS));
        if (received == 1) {
            const bool frame_ready = parser_feed_byte(&s_parser, byte, &frame);
            if (frame_ready) {
                s_count_valid++;
                if (xQueueSend(s_rx_queue, &frame, 0) != pdTRUE) {
                    s_count_dropped++;
                }
            }
        } else {
            if (s_parser.state != STATE_WAIT_START) {
                parser_reset(&s_parser);
                s_count_dropped++;
            }
        }
    }
}