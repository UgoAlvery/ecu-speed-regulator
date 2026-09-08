/*
 * Harness libFuzzer pour protocol_decode().
 *
 * protocol_decode() est la seule fonction du firmware qui consomme un flux
 * d'octets non fiable (UART) — candidate naturelle au fuzzing (roadmap axe 3,
 * item 4). Le harness ne fait aucune supposition sur les données d'entrée :
 * c'est exactement l'exigence du protocole ("toute trame CRC invalide /
 * longueur incohérente / ID inconnu → rejet silencieux, sans crash").
 *
 * protocol.c est une lib pure sans dépendance FreeRTOS/ESP-IDF, donc
 * compilable telle quelle sur machine hôte avec clang -fsanitize=fuzzer.
 */
#include <stddef.h>
#include <stdint.h>

#include "protocol.h"

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    ecu_frame_t out;

    /* out n'est jamais lu s'il n'est pas rempli ; on l'initialise quand
     * même pour que MemorySanitizer (si un jour utilisé) ne râle pas sur
     * un usage conditionnel non initialisé côté harness. */
    out.type = 0;
    out.payload_len = 0;

    (void)protocol_decode(data, size, &out);

    return 0;
}
