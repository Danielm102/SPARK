#include "statemachine.h"

StateMachine_t spark_sm;

uint8_t status_data = 0;
uint8_t selftest_tries = 0;

/* --- Event handlers --- */

static sm_state_t FaultHandler(sm_event_t event) {
    switch (event) {
        case EVENT_FAULT_CLEARED:       return STATE_STANDBY;
        default: return STATE_FAULT;
    }
}

static sm_state_t StartupHandler(sm_event_t event) {
    switch (event) {
        case EVENT_STARTUP_COMPLETE:    return STATE_INIT;
        default: return STATE_STARTUP;
    }
}

static sm_state_t InitHandler(sm_event_t event) {
    switch (event) {
        case EVENT_STEPPER_CONNECTED:   return STATE_STANDBY;
        default: return STATE_INIT;
    }
}

static sm_state_t StandbyHandler(sm_event_t event) {
    switch (event) {
        case EVENT_UVLO:                return STATE_FAULT;
        case EVENT_DRIVER_OVERHEAT:     return STATE_FAULT;
        case EVENT_CMD_ZERO_STEPPER:    return STATE_ZEROING_RETRACT1;
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

static sm_state_t ZeroingRetract1Handler(sm_event_t event) {
    switch (event) {
        case EVENT_UVLO:                return STATE_FAULT;
        case EVENT_DRIVER_OVERHEAT:     return STATE_FAULT;
        case EVENT_CMD_EXIT_MODE:       return STATE_STANDBY;
        case EVENT_STEPPER_STALLED:     return STATE_ZEROING_EXTEND;
        default: return STATE_ZEROING_RETRACT1;
    }
}

static sm_state_t ZeroingExtendHandler(sm_event_t event) {
    switch (event) {
        case EVENT_UVLO:                return STATE_FAULT;
        case EVENT_DRIVER_OVERHEAT:     return STATE_FAULT;
        case EVENT_CMD_EXIT_MODE:       return STATE_STANDBY;
        default: return STATE_ZEROING_EXTEND;
    }
}

static sm_state_t ZeroingRetract2Handler(sm_event_t event) {
    switch (event) {
        case EVENT_UVLO:                return STATE_FAULT;
        case EVENT_DRIVER_OVERHEAT:     return STATE_FAULT;
        case EVENT_CMD_EXIT_MODE:       return STATE_STANDBY;
        case EVENT_STEPPER_STALLED:     return STATE_STANDBY;
        default: return STATE_ZEROING_RETRACT2;
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
static void StartupEntry(StateMachine_t *sm) {
    stepper.active = false;
    Stepper_Wakeup();
    HAL_Delay(5);
}
static void InitEntry(StateMachine_t *sm) {
    stepper.active = false;
    // set stepper neutral angle to current angle so 0 deg target angle corresponds to closed position
    Stepper_Init();
    stepper.neutral_angle = mag_angle_continuous;
}
static void StandbyEntry(StateMachine_t *sm) {
    Stepper_Enable();
    stepper.active = false;
}
static void TargetPositionEntry(StateMachine_t *sm) {
    stepper.active = true;
}
static void TargetSpeedEntry(StateMachine_t *sm) {
    stepper.active = true;
}
static void ZeroingRetract1Entry(StateMachine_t *sm) {
    stepper.active = true;
    Stepper_setStallDetection(DRV_STALL_DETECTION_ON, DRV_STALL_REPORT_ON_NFAULT);
    Stepper_setTargetSpeed(-36); // -36 deg/s
}
static void ZeroingExtendEntry(StateMachine_t *sm) {
    Stepper_moveDeg(10); // extend 10 deg
}
static void ZeroingRetract2Entry(StateMachine_t *sm) {
    Stepper_setTargetSpeed(-36); // -36 deg/s
}
static void FindMaxEntry(StateMachine_t *sm) {}

/* --- Do actions --- */
static void FaultDo(StateMachine_t *sm, uint16_t freq) {}
static void StartupDo(StateMachine_t *sm, uint16_t freq) {
    // Selftest is called with 10 Hz
    if (freq != 10) return;

    if ((status_data & 0x03) == 0x03) {
        // all selftests passed
        StateMachine_Dispatch(sm, EVENT_STARTUP_COMPLETE);
        status_data |= (1 << 7);
    } else {
        selftest_tries++;
        status_data |= (Stepper_SelfTest() << 0);
        status_data |= (AS5600_SelfTest() << 1);
    }
}
static void InitDo(StateMachine_t *sm, uint16_t freq) {
    if (freq != 10) return;

    Stepper_getFullStatus();
    if (!DRV_status.FAULT && voltage_driver >= 12.0f) {
        StateMachine_Dispatch(sm, EVENT_STEPPER_CONNECTED);
    }
}
static void StandbyDo(StateMachine_t *sm, uint16_t freq) {}
static void TargetPositionDo(StateMachine_t *sm, uint16_t freq) {}
static void TargetSpeedDo(StateMachine_t *sm, uint16_t freq) {}
static void ZeroingRetract1Do(StateMachine_t *sm, uint16_t freq) {}
static void ZeroingExtendDo(StateMachine_t *sm, uint16_t freq) {}
static void ZeroingRetract2Do(StateMachine_t *sm, uint16_t freq) {}
static void FindMaxDo(StateMachine_t *sm, uint16_t freq) {}

/* --- Exit actions --- */
static void FaultExit(StateMachine_t *sm) {}
static void StartupExit(StateMachine_t *sm) {}
static void InitExit(StateMachine_t *sm) {}
static void StandbyExit(StateMachine_t *sm) {}
static void TargetPositionExit(StateMachine_t *sm) {}
static void TargetSpeedExit(StateMachine_t *sm) {}
static void ZeroingRetract1Exit(StateMachine_t *sm) {}
static void ZeroingExtendExit(StateMachine_t *sm) {}
static void ZeroingRetract2Exit(StateMachine_t *sm) {
    // set stepper neutral angle to current angle so 0 deg target angle corresponds to closed position
    stepper.neutral_angle = mag_angle_continuous;
}
static void FindMaxExit(StateMachine_t *sm) {}

/* --- Lookup tables for state functions --- */
static StateHandler_t stateHandlerTable[STATE_MAX] = {
    FaultHandler,
    StartupHandler,
    InitHandler,
    StandbyHandler,
    TargetPositionHandler,
    TargetSpeedHandler,
    ZeroingRetract1Handler,
    ZeroingExtendHandler,
    ZeroingRetract2Handler,
    FindMaxHandler
};

static StateEntry_t stateEntryTable[STATE_MAX] = {
    FaultEntry,
    StartupEntry,
    InitEntry,
    StandbyEntry,
    TargetPositionEntry,
    TargetSpeedEntry,
    ZeroingRetract1Entry,
    ZeroingExtendEntry,
    ZeroingRetract2Entry,
    FindMaxEntry
};

static StateDo_t stateDoTable[STATE_MAX] = {
    FaultDo,
    StartupDo,
    InitDo,
    StandbyDo,
    TargetPositionDo,
    TargetSpeedDo,
    ZeroingRetract1Do,
    ZeroingExtendDo,
    ZeroingRetract2Do,
    FindMaxDo
};

static StateExit_t stateExitTable[STATE_MAX] = {
    FaultExit,
    StartupExit,
    InitExit,
    StandbyExit,
    TargetPositionExit,
    TargetSpeedExit,
    ZeroingRetract1Exit,
    ZeroingExtendExit,
    ZeroingRetract2Exit,
    FindMaxExit
};

/* --- Lookup tables for timing constraints --- */
uint32_t minEventDelayTable[EVENT_MAX] = {
    0,
    0,
    0,
    0,
    1000,
    0,
    0,
    0,
    0,
    0,
    0,
    500
};

uint32_t maxEventDelayTable[STATE_MAX] = {
    0,
    0,
    0,
    0,
    0,
    0,
    5000,
    0,
    1000,
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