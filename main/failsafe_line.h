#ifndef ECU_FAILSAFE_LINE_H
#define ECU_FAILSAFE_LINE_H

/* Machine à états de la ligne failsafe GPIO (contact NC) — lib pure, sans
 * FreeRTOS ni driver, testée en host. task_failsafe lui fournit les
 * événements (front montant, niveau lu au polling) et exécute les actions
 * retournées (déclencher le failsafe, déverrouiller). */

#include <stdbool.h>
#include <stdint.h>

/* Polls consécutifs à « ligne fermée » requis avant déverrouillage
 * (anti-rebond à la remise du jumper). Le déclenchement est immédiat. */
#define FAILSAFE_LINE_CLOSED_POLLS_TO_UNLOCK  2u

typedef enum {
    FAILSAFE_LINE_NONE,     /* rien à faire                                   */
    FAILSAFE_LINE_TRIP,     /* transition vers failsafe : verrou + OFF + ALARM */
    FAILSAFE_LINE_UNLOCK,   /* ligne refermée et stable : lever le verrou      */
} failsafe_line_action_t;

typedef struct {
    bool     latched;       /* failsafe GPIO actif (verrou posé)  */
    uint32_t closed_polls;  /* polls consécutifs ligne fermée     */
} failsafe_line_t;

void failsafe_line_init(failsafe_line_t *line);

/* Front montant (ISR) : TRIP au premier front, NONE tant que verrouillé. */
failsafe_line_action_t failsafe_line_on_edge(failsafe_line_t *line);

/* Niveau lu au polling (ou au boot) : open = ligne ouverte (niveau 1). */
failsafe_line_action_t failsafe_line_on_level(failsafe_line_t *line, bool open);

#endif /* ECU_FAILSAFE_LINE_H */
