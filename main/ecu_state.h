//
// Created by ugoal on 29/05/2026.
//

#ifndef ECU_ECU_STATE_H
#define ECU_ECU_STATE_H

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ecu_command.h"   /* ecu_mode_t, cœur mode / commande / verrou (pur) */

typedef struct {
    ecu_command_t cmd;        /* mode, output, failsafe_locked */
    float         setpoint;
    float         speed;
    TickType_t    last_rx_tick;
} ecu_state_t;

void ecu_state_init(void);

void ecu_state_set_setpoint(float setpoint);
void ecu_state_set_speed(float speed);
void ecu_state_set_last_rx_tick(TickType_t tick);
void ecu_state_set_failsafe_locked(bool locked);

/* Lecture-modification-écriture : toujours une fonction dédiée, sous une
 * seule prise du mutex (règle du module, cf. technical_reference.md).
 *
 * try_set_mode : refuse ECU_MODE_AUTO tant que failsafe_locked est vrai.
 * Deux appels séparés (lecture du verrou puis set_mode) laisseraient
 * task_failsafe verrouiller entre les deux et l'AUTO passerait quand même.
 * Retourne false si la demande est refusée (état inchangé). */
bool ecu_state_try_set_mode(ecu_mode_t mode);

/* commit_output : relit le mode, fixe la commande (requested en AUTO, 0
 * sinon) ET la met en queue via emit, le tout sous une seule prise du mutex.
 * emit doit être non bloquant (timeout 0). Retourne false si la trame n'a
 * pas pu être mise en queue. */
bool ecu_state_commit_output(float requested, ecu_output_emit_fn emit, void *ctx);

/* enter_failsafe : verrou (si lock), output = 0, mode = OFF, puis OUTPUT(0)
 * en FIN de queue via emit, sous une seule prise du mutex. Comme toute
 * commande non nulle est mise en queue sous ce même mutex, elle précède
 * forcément cet OUTPUT(0) dans queue_tx (rta_fmea.md §6.1 bis). Retourne
 * false si OUTPUT(0) n'a pas pu être mis en queue (l'état passe en OFF). */
bool ecu_state_enter_failsafe(bool lock, ecu_output_emit_fn emit, void *ctx);

ecu_mode_t ecu_state_get_mode(void);
float      ecu_state_get_setpoint(void);
float      ecu_state_get_speed(void);
float      ecu_state_get_output(void);
TickType_t ecu_state_get_last_rx_tick(void);

#endif //ECU_ECU_STATE_H