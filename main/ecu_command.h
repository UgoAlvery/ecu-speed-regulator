#ifndef ECU_COMMAND_H
#define ECU_COMMAND_H

/* Cœur « mode / commande moteur / verrou failsafe » — lib pure, sans FreeRTOS
 * ni driver, testée en host. ecu_state appelle chaque fonction sous UNE prise
 * de mutex_ecu_state : c'est ce qui rend chacune atomique sur cible.
 *
 * L'émission d'OUTPUT est injectée (callback) : décider la valeur ET la mettre
 * en queue dans la même section critique est ce qui ordonne les trames OUTPUT
 * par rapport au passage en failsafe (rta_fmea.md §6.1 bis). */

#include <stdbool.h>

typedef enum {
    ECU_MODE_OFF    = 0,
    ECU_MODE_MANUAL = 1,
    ECU_MODE_AUTO   = 2,
} ecu_mode_t;

typedef struct {
    ecu_mode_t mode;
    float      output;
    bool       failsafe_locked;   /* ligne failsafe GPIO ouverte : AUTO interdit */
} ecu_command_t;

/* Met une trame OUTPUT en queue, sans bloquer. Retourne false si la queue est
 * pleine (trame non émise). ctx : contexte opaque de l'appelant. */
typedef bool (*ecu_output_emit_fn)(float output, void *ctx);

void ecu_command_init(ecu_command_t *cmd);

/* Refuse ECU_MODE_AUTO tant que failsafe_locked ; retourne false si refusé. */
bool ecu_command_try_set_mode(ecu_command_t *cmd, ecu_mode_t mode);

/* Commande retenue = requested en AUTO, 0 sinon ; puis émission. Retourne le
 * résultat de emit (false si non émise ou argument NULL). */
bool ecu_command_commit_output(ecu_command_t *cmd, float requested,
                               ecu_output_emit_fn emit, void *ctx);

/* [verrou si lock] + output = 0 + mode OFF, puis émission d'OUTPUT(0).
 * L'état passe en failsafe même si l'émission échoue. Retourne le résultat
 * de emit. */
bool ecu_command_enter_failsafe(ecu_command_t *cmd, bool lock,
                                ecu_output_emit_fn emit, void *ctx);

void ecu_command_set_failsafe_locked(ecu_command_t *cmd, bool locked);

#endif /* ECU_COMMAND_H */
