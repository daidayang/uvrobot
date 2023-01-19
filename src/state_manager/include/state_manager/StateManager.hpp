#ifndef _STATE_MANAGER_HPP_
#define _STATE_MANAGER_HPP_

#include <iostream>
#include <map>

#include <state_manager/StateBase.hpp>
#include <state_manager/IdleState.hpp>
#include <state_manager/DisinfectionState.hpp>
#include <state_manager/ChargeState.hpp>
#include <state_manager/FaultState.hpp>

class StateManager
{
public:
    StateManager();
    ~StateManager();

    void changeState(StateBase *state);

    StateBase* getNextState(int error_flag, int policy_flag, int charge_flag);
    StateBase *getCurrentState();
    StateBase *getLastState();

private:
    StateBase *m_currentState;
    StateBase *m_lastState;
    std::map<int, StateBase*> m_modes;
};

#endif // _STATE_MANAGER_HPP_
