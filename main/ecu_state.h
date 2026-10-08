//
// Created by ugoal on 29/05/2026.
//

#ifndef ECU_ECU_STATE_H
#define ECU_ECU_STATE_H

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef enum {
    ECU_MODE_OFF    = 0,
    ECU_MODE_MANUAL = 1,
    ECU_MODE_AUTO   = 2,
} ecu_mode_t;

typedef struct {
    ecu_mode_t mode;
    float      setpoint;
    float      speed;
    float      output;
    TickType_t last_rx_tick;
    bool       failsafe_locked;  /* ligne failsafe GPIO ouverte : AUTO interdit */
} ecu_state_t;

void ecu_state_init(void);

void ecu_state_set_mode(ecu_mode_t mode);
void ecu_state_set_setpoint(float setpoint);
void ecu_state_set_speed(float speed);
void ecu_state_set_output(float output);
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

ecu_mode_t ecu_state_get_mode(void);
float      ecu_state_get_setpoint(void);
float      ecu_state_get_speed(void);
float      ecu_state_get_output(void);
TickType_t ecu_state_get_last_rx_tick(void);

#endif //ECU_ECU_STATE_H