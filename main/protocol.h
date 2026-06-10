#ifndef PROTOCOL_H
#define PROTOCOL_H

#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MSG_SETPOINT  0x01
#define MSG_SPEED     0x02
#define MSG_MODE_SET  0x05
#define MSG_OUTPUT    0x80
#define MSG_STATS     0x83
#define MSG_ALARM     0x85
#define MSG_DBG       0xFF

#define PROTOCOL_START_BYTE        0xAA
#define PROTOCOL_MAX_PAYLOAD_SIZE  128
#define PROTOCOL_MAX_FRAME_SIZE    (1 + 2 + 1 + PROTOCOL_MAX_PAYLOAD_SIZE + 1)
//                                  AA  LEN TYPE      PAYLOAD              CRC

typedef struct {
    uint8_t  type;
    uint8_t  payload[PROTOCOL_MAX_PAYLOAD_SIZE];
    uint16_t payload_len;  // = LEN - 1
} ecu_frame_t;

uint8_t protocol_compute_crc(const uint8_t *data, size_t len);

size_t protocol_encode(uint8_t *dst, uint8_t type,
    const uint8_t *payload, uint16_t payload_len);

bool protocol_decode(const uint8_t *frame, size_t frame_len,
    ecu_frame_t *out);

#endif