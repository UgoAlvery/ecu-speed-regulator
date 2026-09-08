#include "protocol.h"
#include <string.h>


/* CRC-16/CCITT, polynôme 0x1021, init 0x0000 (variante XMODEM).
 * Choix justifié : détecte toute rafale d'erreurs ≤ 16 bits et toute erreur
 * sur 1 ou 2 bits, contrairement au XOR (transparent aux erreurs sur un
 * nombre pair de bits inversés). Distance de Hamming supérieure sur les
 * trames courtes (<256 octets) face à CRC-16/IBM (0x8005). */
uint16_t protocol_compute_crc(const uint8_t *data, size_t len) {
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000u)
                ? (uint16_t)((crc << 1) ^ 0x1021u)
                : (uint16_t)(crc << 1);
        }
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
        memcpy(&dst[idx], payload, payload_len);
        idx += payload_len;
    }

    const uint16_t crc = protocol_compute_crc(&dst[1], idx - 1);
    dst[idx++] = (uint8_t)(crc & 0xFF);
    dst[idx++] = (uint8_t)(crc >> 8);
    return idx;
}

bool protocol_decode(const uint8_t *frame, const size_t frame_len,
    ecu_frame_t *out) {
    if (!frame || !out || !frame_len) return false;
    if (frame_len < 6) return false;  /* AA + LEN(2) + TYPE(1) + CRC16(2) */
    if (frame[0] != PROTOCOL_START_BYTE) return false;
    const uint16_t len_field = (uint16_t)frame[1] | ((uint16_t)frame[2] << 8);

    if (len_field > frame_len) return false;

    const size_t expected_frame_len = 1 + 2 + len_field + 2;
    if (frame_len != expected_frame_len) return false;

    const uint16_t payload_len = len_field - 1;
    if (payload_len > PROTOCOL_MAX_PAYLOAD_SIZE) return false;

    const size_t crc_zone_len = frame_len - 1 - 2;
    const uint16_t expected_crc = protocol_compute_crc(&frame[1], crc_zone_len);
    const uint16_t received_crc = (uint16_t)frame[frame_len - 2]
                                 | ((uint16_t)frame[frame_len - 1] << 8);
    if (expected_crc != received_crc) return false;

    out->type = frame[3];
    out->payload_len = payload_len;
    if (payload_len > 0) {
        memcpy(&out->payload, &frame[4], out->payload_len);
    }

    return true;
}
