#include "protocol.h"
#include <string.h>


uint8_t protocol_compute_crc(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

size_t protocol_encode(uint8_t *dst, const uint8_t type,
                       const uint8_t *payload, const uint16_t payload_len) {
    if (!dst) return 0;
    if (payload_len > PROTOCOL_MAX_PAYLOAD_SIZE) return 0;

    const uint16_t len_field = payload_len + 1;

    size_t idx = 0;
    dst[idx++] = PROTOCOL_START_BYTE;
    dst[idx++] = (uint8_t)(len_field & 0xFF);
    dst[idx++] = (uint8_t)(len_field >> 8);
    dst[idx++] = type;

    if (payload && payload_len > 0) {
        memcpy(&dst[idx],payload,payload_len);
        idx += payload_len;
    }

    const uint8_t crc = protocol_compute_crc(&dst[1], idx -1);
    dst[idx++] = crc;
    return idx;
}

bool protocol_decode(const uint8_t *frame, const size_t frame_len,
    ecu_frame_t *out) {
    if (!frame || !frame_len) return false;
    if (frame_len < 5) return false;
    if (frame[0] != PROTOCOL_START_BYTE) return false;
    const uint16_t len_field = (uint16_t)frame[1] | ((uint16_t)frame[2] << 8);

    if (len_field > frame_len) return false;

    const size_t expected_frame_len = 1 + 2 +len_field + 1;
    if (frame_len != expected_frame_len) return false;

    const uint16_t payload_len = len_field - 1;
    if (payload_len > PROTOCOL_MAX_PAYLOAD_SIZE) return false;

    const size_t crc_zone_len = frame_len - 1 - 1;
    const uint8_t expected_crc = protocol_compute_crc(&frame[1], crc_zone_len);
    const uint8_t received_crc = frame[frame_len - 1];
    if (expected_crc != received_crc) return false;

    out->type = frame[3];
    out->payload_len = payload_len;
    if (payload_len > 0) {
        memcpy(&out->payload, &frame[4], out->payload_len);
    }

    return true;
}