/*
 * Génère un corpus de départ pour le fuzzing de protocol_decode(), à partir
 * de trames valides produites par protocol_encode() (dont on sait, via
 * test/unit/test_protocol.c, qu'il produit un encodage correct). Un corpus
 * de trames structurellement valides fait converger le fuzzer bien plus vite
 * qu'un corpus vide : libFuzzer part de mutations d'octets déjà "proches" du
 * format attendu (START, LEN, TYPE, CRC) plutôt que de devoir le redécouvrir
 * par recherche aléatoire pure.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "protocol.h"

static void write_seed(const char *dir, const char *name,
                        const uint8_t *buf, size_t len)
{
    char path[256];
    snprintf(path, sizeof(path), "%s/%s", dir, name);

    FILE *f = fopen(path, "wb");
    if (!f) {
        perror(path);
        return;
    }
    fwrite(buf, 1, len, f);
    fclose(f);
}

int main(void)
{
    const char *dir = "corpus";
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len;
    char name[64];
    uint16_t nonce = 1;  /* valeur arbitraire — protocol_decode() ne vérifie pas le nonce */

    /* Une trame vide par type de message connu. */
    const uint8_t types[] = {
        MSG_SETPOINT, MSG_SPEED, MSG_MODE_SET,
        MSG_OUTPUT, MSG_STATS, MSG_ALARM, MSG_DBG,
    };
    for (size_t i = 0; i < sizeof(types); i++) {
        len = protocol_encode(buf, types[i], nonce++, NULL, 0);
        snprintf(name, sizeof(name), "empty_type_%02x", types[i]);
        write_seed(dir, name, buf, len);
    }

    /* Petit payload. */
    const uint8_t payload_small[4] = {0x01, 0x02, 0x03, 0x04};
    len = protocol_encode(buf, MSG_SETPOINT, nonce++, payload_small, sizeof(payload_small));
    write_seed(dir, "small_payload", buf, len);

    /* Payload maximal (borne PROTOCOL_MAX_PAYLOAD_SIZE). */
    uint8_t payload_max[PROTOCOL_MAX_PAYLOAD_SIZE];
    for (size_t i = 0; i < sizeof(payload_max); i++) {
        payload_max[i] = (uint8_t)i;
    }
    len = protocol_encode(buf, MSG_DBG, nonce++, payload_max, sizeof(payload_max));
    write_seed(dir, "max_payload", buf, len);

    /* Trame valide avec CRC volontairement corrompu : forme correcte,
     * contenu invalide — utile pour explorer autour du chemin de rejet CRC. */
    len = protocol_encode(buf, MSG_STATS, nonce++, payload_small, sizeof(payload_small));
    buf[len - 1] ^= 0xFF;
    write_seed(dir, "corrupted_crc", buf, len);

    /* LEN incohérent (gonflé au-delà de la trame réelle). */
    len = protocol_encode(buf, MSG_OUTPUT, nonce++, payload_small, sizeof(payload_small));
    buf[1] = (uint8_t)0xFF;
    write_seed(dir, "inflated_len", buf, len);

    /* Mauvais octet START. */
    len = protocol_encode(buf, MSG_ALARM, nonce++, payload_small, sizeof(payload_small));
    buf[0] = 0x00;
    write_seed(dir, "bad_start", buf, len);

    printf("corpus genere dans %s/\n", dir);
    return 0;
}
