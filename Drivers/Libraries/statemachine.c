#include "statemachine.h"

StateMachine_t flight_sm;

/* --- Event handlers --- */

static sm_state_t StartupHandler(sm_event_t event) {
    switch (event) {

        default: return STATE_STARTUP;
    }
}

static sm_state_t InitHandler(sm_event_t event) {
    switch (event) {

        default: return STATE_INIT;
    }
}

/* --- Entry actions --- */
static void StartupEntry(StateMachine_t *sm) {}
static void InitEntry(StateMachine_t *sm) {}

/* --- Do actions --- */
static void StartupDo(StateMachine_t *sm, uint16_t freq) {}
static void InitDo(StateMachine_t *sm, uint16_t freq) {}

/* --- Exit actions --- */
static void StartupExit(StateMachine_t *sm) {}
static void InitExit(StateMachine_t *sm) {}

/* --- Lookup tables for state functions --- */
static StateHandler_t stateHandlerTable[STATE_MAX] = {
    StartupHandler,
    InitHandler
};

static StateEntry_t stateEntryTable[STATE_MAX] = {
    StartupEntry,
    InitEntry
};

static StateDo_t stateDoTable[STATE_MAX] = {
    StartupDo,
    InitDo
};

static StateExit_t stateExitTable[STATE_MAX] = {
    StartupExit,
    InitExit
};

/* --- Lookup tables for timing constraints --- */
uint32_t minEventDelayTable[EVENT_MAX] = {
    0,
    0
};

uint32_t maxEventDelayTable[STATE_MAX] = {
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