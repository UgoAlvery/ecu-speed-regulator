#include "failsafe_line.h"

#include <stddef.h>

void failsafe_line_init(failsafe_line_t *line)
{
    if (line == NULL) return;
    line->latched      = false;
    line->closed_polls = 0;
}

static failsafe_line_action_t trip(failsafe_line_t *line)
{
    line->closed_polls = 0;
    if (line->latched) {
        return FAILSAFE_LINE_NONE;   /* debounce : une seule ALARM par ouverture */
    }
    line->latched = true;
    return FAILSAFE_LINE_TRIP;
}

failsafe_line_action_t failsafe_line_on_edge(failsafe_line_t *line)
{
    if (line == NULL) return FAILSAFE_LINE_NONE;
    return trip(line);
}

failsafe_line_action_t failsafe_line_on_level(failsafe_line_t *line, const bool open)
{
    if (line == NULL) return FAILSAFE_LINE_NONE;

    if (open) {
        return trip(line);
    }

    if (!line->latched) {
        return FAILSAFE_LINE_NONE;
    }
    if (++line->closed_polls < FAILSAFE_LINE_CLOSED_POLLS_TO_UNLOCK) {
        return FAILSAFE_LINE_NONE;
    }
    line->latched      = false;
    line->closed_polls = 0;
    return FAILSAFE_LINE_UNLOCK;
}
