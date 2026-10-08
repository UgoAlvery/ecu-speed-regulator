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

/* Valeur max d'un MODE_SET (0=OFF, 1=MANUAL, 2=AUTO, cf. ecu_mode_t). */
#define PROTOCOL_MODE_MAX  2u

/* Bornes de plausibilité (km/h), inclusives. SPEED_MAX = SETPOINT_MAX + 10 %,
 * marge pour un dépassement transitoire de consigne. Hors bornes : trame rejetée,
 * dernière valeur valide conservée, compteur rx_out_of_range (rta_fmea R-7). */
#define PROTOCOL_SETPOINT_MIN   0.0f
#define PROTOCOL_SETPOINT_MAX   220.0f
#define PROTOCOL_SPEED_MIN      0.0f
#define PROTOCOL_SPEED_MAX      242.0f

#define PROTOCOL_START_BYTE        0xAA
#define PROTOCOL_MAX_PAYLOAD_SIZE  128
#define PROTOCOL_NONCE_SIZE        2
#define PROTOCOL_MAX_FRAME_SIZE    (1 + 2 + 1 + PROTOCOL_NONCE_SIZE + PROTOCOL_MAX_PAYLOAD_SIZE + 2)
//                                  AA  LEN TYPE   NONCE         PAYLOAD            CRC16

typedef struct {
    uint8_t  type;
    uint16_t nonce;
    uint8_t  payload[PROTOCOL_MAX_PAYLOAD_SIZE];
    uint16_t payload_len;  // = LEN - 1 - PROTOCOL_NONCE_SIZE
} ecu_frame_t;

/* Authentification légère par nonce anti-rejeu : lib pure, aucun état global —
 * le contexte (dernier nonce accepté) est passé par pointeur, comme pid_t.
 * Un contexte par flux/direction à protéger (ex : un par task_rx). */
typedef struct {
    uint16_t last_nonce;
} protocol_nonce_ctx_t;

uint16_t protocol_compute_crc(const uint8_t *data, size_t len);

size_t protocol_encode(uint8_t *dst, uint8_t type, uint16_t nonce,
    const uint8_t *payload, uint16_t payload_len);

bool protocol_decode(const uint8_t *frame, size_t frame_len,
    ecu_frame_t *out);

void protocol_nonce_ctx_init(protocol_nonce_ctx_t *ctx);

/* Accepte ssi nonce est strictement postérieur au dernier nonce accepté, au
 * sens de l'arithmétique modulaire 16 bits (tolère le wraparound, comme les
 * numéros de séquence TCP) : rejette rejeu, duplication et trames retardées
 * hors ordre. Met à jour ctx uniquement en cas d'acceptation. */
bool protocol_nonce_check_and_update(protocol_nonce_ctx_t *ctx, uint16_t nonce);

/* Validation applicative des payloads reçus (trame déjà décodée, CRC OK).
 * Retournent false — et ne touchent pas *out — si la longueur du payload n'est
 * pas exactement celle attendue pour le type, ou si la valeur est hors domaine.
 * SETPOINT / SPEED : float32 LE, 4 octets, valeur finie (ni NaN ni Inf).
 * MODE_SET         : uint8, 1 octet, valeur <= PROTOCOL_MODE_MAX. */
bool protocol_parse_f32(const ecu_frame_t *frame, float *out);
bool protocol_parse_mode(const ecu_frame_t *frame, uint8_t *out);

/* Comme protocol_parse_f32, puis contrôle de plausibilité min <= v <= max.
 * Distingue la trame malformée (INVALID : longueur, NaN/Inf) de la valeur
 * bien formée mais hors bornes (OUT_OF_RANGE) pour les compter séparément.
 * *out n'est écrit que si le résultat est PROTOCOL_VALUE_OK. */
typedef enum {
    PROTOCOL_VALUE_OK,
    PROTOCOL_VALUE_INVALID,
    PROTOCOL_VALUE_OUT_OF_RANGE,
} protocol_value_status_t;

protocol_value_status_t protocol_parse_f32_bounded(const ecu_frame_t *frame,
                                                   float min, float max, float *out);

#endif
