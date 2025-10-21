#include "statemachine.h"

StateMachine_t flight_sm;

/* --- Event handlers --- */

static sm_state_t FaultHandler(sm_event_t event) {
    switch (event) {
        case EVENT_FAULT_CLEARED:       return STATE_STANDBY;
        default: return STATE_FAULT;
    }
}

static sm_state_t StartupHandler(sm_event_t event) {
    switch (event) {
        case EVENT_DRIVER_OVERHEAT:     return STATE_FAULT;
        case EVENT_STARTUP_COMPLETE:    return STATE_INIT;
        default: return STATE_STARTUP;
    }
}

static sm_state_t InitHandler(sm_event_t event) {
    switch (event) {
        case EVENT_DRIVER_OVERHEAT:     return STATE_FAULT;
        case EVENT_STEPPER_CONNECTED:   return STATE_STANDBY;
        default: return STATE_INIT;
    }
}

static sm_state_t StandbyHandler(sm_event_t event) {
    switch (event) {
        case EVENT_UVLO:                return STATE_FAULT;
        case EVENT_DRIVER_OVERHEAT:     return STATE_FAULT;
        case EVENT_CMD_ZERO_STEPPER:    return STATE_ZEROING;
        case EVENT_CMD_TARGET_POSITION: return STATE_TARGET_POSITION;
        case EVENT_CMD_TARGET_SPEED:    return STATE_TARGET_SPEED;
        case EVENT_CMD_FIND_MAX:        return STATE_FIND_MAX;
        default: return STATE_STANDBY;
    }
}

static sm_state_t TargetPositionHandler(sm_event_t event) {
    switch (event) {
        case EVENT_UVLO:                return STATE_FAULT;
        case EVENT_DRIVER_OVERHEAT:     return STATE_FAULT;
        case EVENT_CMD_EXIT_MODE:       return STATE_STANDBY;
        default: return STATE_TARGET_POSITION;
    }
}

static sm_state_t TargetSpeedHandler(sm_event_t event) {
    switch (event) {
        case EVENT_UVLO:                return STATE_FAULT;
        case EVENT_DRIVER_OVERHEAT:     return STATE_FAULT;
        case EVENT_CMD_EXIT_MODE:       return STATE_STANDBY;
        default: return STATE_TARGET_SPEED;
    }
}

static sm_state_t ZeroingHandler(sm_event_t event) {
    switch (event) {
        case EVENT_UVLO:                return STATE_FAULT;
        case EVENT_DRIVER_OVERHEAT:     return STATE_FAULT;
        case EVENT_CMD_EXIT_MODE:       return STATE_STANDBY;
        case EVENT_ZEROING_COMPLETE:    return STATE_STANDBY;
        default: return STATE_ZEROING;
    }
}

static sm_state_t FindMaxHandler(sm_event_t event) {
    switch (event) {
        case EVENT_UVLO:                return STATE_FAULT;
        case EVENT_DRIVER_OVERHEAT:     return STATE_FAULT;
        case EVENT_CMD_EXIT_MODE:       return STATE_STANDBY;
        case EVEMT_FIND_MAX_COMPLETE:   return STATE_STANDBY;
        default: return STATE_FIND_MAX;
    }
}

/* --- Entry actions --- */
static void FaultEntry(StateMachine_t *sm) {}
static void StartupEntry(StateMachine_t *sm) {}
static void InitEntry(StateMachine_t *sm) {}
static void StandbyEntry(StateMachine_t *sm) {}
static void TargetPositionEntry(StateMachine_t *sm) {}
static void TargetSpeedEntry(StateMachine_t *sm) {}
static void ZeroingEntry(StateMachine_t *sm) {}
static void FindMaxEntry(StateMachine_t *sm) {}

/* --- Do actions --- */
static void FaultDo(StateMachine_t *sm, uint16_t freq) {}
static void StartupDo(StateMachine_t *sm, uint16_t freq) {}
static void InitDo(StateMachine_t *sm, uint16_t freq) {}
static void StandbyDo(StateMachine_t *sm, uint16_t freq) {}
static void TargetPositionDo(StateMachine_t *sm, uint16_t freq) {}
static void TargetSpeedDo(StateMachine_t *sm, uint16_t freq) {}
static void ZeroingDo(StateMachine_t *sm, uint16_t freq) {}
static void FindMaxDo(StateMachine_t *sm, uint16_t freq) {}

/* --- Exit actions --- */
static void FaultExit(StateMachine_t *sm) {}
static void StartupExit(StateMachine_t *sm) {}
static void InitExit(StateMachine_t *sm) {}
static void StandbyExit(StateMachine_t *sm) {}
static void TargetPositionExit(StateMachine_t *sm) {}
static void TargetSpeedExit(StateMachine_t *sm) {}
static void ZeroingExit(StateMachine_t *sm) {}
static void FindMaxExit(StateMachine_t *sm) {}

/* --- Lookup tables for state functions --- */
static StateHandler_t stateHandlerTable[STATE_MAX] = {
    FaultHandler,
    StartupHandler,
    InitHandler,
    StandbyHandler,
    TargetPositionHandler,
    TargetSpeedHandler,
    ZeroingHandler,
    FindMaxHandler
};

static StateEntry_t stateEntryTable[STATE_MAX] = {
    FaultEntry,
    StartupEntry,
    InitEntry,
    StandbyEntry,
    TargetPositionEntry,
    TargetSpeedEntry,
    ZeroingEntry,
    FindMaxEntry
};

static StateDo_t stateDoTable[STATE_MAX] = {
    FaultDo,
    StartupDo,
    InitDo,
    StandbyDo,
    TargetPositionDo,
    TargetSpeedDo,
    ZeroingDo,
    FindMaxDo
};

static StateExit_t stateExitTable[STATE_MAX] = {
    FaultExit,
    StartupExit,
    InitExit,
    StandbyExit,
    TargetPositionExit,
    TargetSpeedExit,
    ZeroingExit,
    FindMaxExit
};

/* --- Lookup tables for timing constraints --- */
uint32_t minEventDelayTable[EVENT_MAX] = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
};

uint32_t maxEventDelayTable[STATE_MAX] = {
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0
};

/* --- Action handler functions --- */
static void StateEntryHandler(StateMachine_t *sm, sm_state_t state) {
    // set entry timestamp
    sm->timestamp_us = uwTick;

    // handle entry for each state
    stateEntryTable[state](sm);
}

static void StateExitHandler(StateMachine_t *sm, sm_state_t state) {
    stateExitTable[state](sm);
}

static void StateDoHandler(StateMachine_t *sm, sm_state_t state, uint16_t freq) {
    stateDoTable[state](sm, freq);
}

/* --- User functions --- */
void StateMachine_Init(StateMachine_t *sm, sm_state_t initialState) {
    sm->currentState = initialState;

    // call entry action for initial state
    StateEntryHandler(sm, sm->currentState);
}

void StateMachine_Dispatch(StateMachine_t *sm, sm_event_t event) {
    if (sm->currentState >= STATE_MAX) return;

    // TODO: store event on Flash & SD
    
    // store old flight state
    sm_state_t oldState = sm->currentState;

    // handle event and retrieve new flight state
    sm_state_t newState = stateHandlerTable[oldState](event);

    // prevent exit and entry actions if no state change
    if (newState == oldState || newState >= STATE_MAX) return;

    // don't update state if minimum entry time delay for event hasn't elapsed yet
    if (minEventDelayTable[event] > (uwTick - sm->timestamp_us)) return;
    
    // exit old state
    StateExitHandler(sm, oldState);

    // update flight state
    sm->currentState = newState;

    // stop timer
    HAL_TIM_Base_Stop_IT(&htim7);

    // start timer to enforce maximum delay until event
    if ((tim7_target_ms = maxEventDelayTable[newState])) {
        HAL_TIM_Base_Start_IT(&htim7);
    }

    // enter new state
    StateEntryHandler(sm, newState);
}

void StateMachine_ForceState(StateMachine_t *sm, sm_state_t newState) {
    if (sm->currentState >= STATE_MAX) return;

    // TODO: store command on Flash & SD
    
    // store old flight state
    sm_state_t oldState = sm->currentState;

    // prevent exit and entry actions if no state change
    if (newState == oldState || newState >= STATE_MAX) return;
    
    // exit old state
    StateExitHandler(sm, oldState);

    // update flight state
    sm->currentState = newState;

    // stop timer
    HAL_TIM_Base_Stop_IT(&htim7);

    // start timer to enforce maximum delay until event
    if ((tim7_target_ms = maxEventDelayTable[newState])) {
        HAL_TIM_Base_Start_IT(&htim7);
    }

    // enter new state
    StateEntryHandler(sm, newState);
}

void StateMachine_DoActions(StateMachine_t *sm, uint16_t freq) {
    // perform standard state actions
    StateDoHandler(sm, sm->currentState, freq);
}