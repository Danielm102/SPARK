#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "main.h"


/* --- Define minimum Event delay times in ms --- */
// #define MIN_DELAY_UNTIL_FILTER_CONVERGED


/* --- Define maximum Event delay times in ms --- */
// #define MAX_DELAY_UNTIL_


/* --- Define all possible states --- */
typedef enum {
    STATE_FAULT,
    STATE_STARTUP,
    STATE_INIT,
    STATE_STANDBY,
    STATE_TARGET_POSITION,
    STATE_TARGET_SPEED,
    STATE_ZEROING,
    STATE_FIND_MAX,

    STATE_MAX
} sm_state_t;

/* --- Define all possible events --- */
typedef enum {
    EVENT_UVLO,                 // under-voltage lockout detected
    EVENT_DRIVER_OVERHEAT,      // motor driver overheat detected
    EVENT_FAULT_CLEARED,        // fault condition cleared
    EVENT_STARTUP_COMPLETE,     // startup sequence complete
    EVENT_STEPPER_CONNECTED,
    EVENT_CMD_ZERO_STEPPER,
    EVENT_CMD_TARGET_POSITION,
    EVENT_CMD_TARGET_SPEED,
    EVENT_CMD_FIND_MAX,
    EVENT_CMD_EXIT_MODE,
    EVENT_ZEROING_COMPLETE,
    EVEMT_FIND_MAX_COMPLETE,

    EVENT_MAX
} sm_event_t;

typedef struct {
    sm_state_t currentState;  // Holds the current state
    uint32_t timestamp_us;    // Holds time of entering current state
} StateMachine_t;

typedef sm_state_t (*StateHandler_t)(sm_event_t event);
typedef void (*StateEntry_t)(StateMachine_t *sm);
typedef void (*StateDo_t)(StateMachine_t *sm, uint16_t freq);
typedef void (*StateExit_t)(StateMachine_t *sm);

/* --- Extern variables used in the state machine Entry, Do and Exit functions --- */
extern TIM_HandleTypeDef htim7;
extern uint32_t tim7_target_ms;

/* --- Function declarations --- */
void StateMachine_Init(StateMachine_t *sm, sm_state_t initialState);
void StateMachine_Dispatch(StateMachine_t *sm, sm_event_t event);
void StateMachine_ForceState(StateMachine_t *sm, sm_state_t newState);
void StateMachine_DoActions(StateMachine_t *sm, uint16_t freq);

#endif // STATEMACHINE_H