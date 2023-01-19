#include <state_manager/StateManager.hpp>

StateManager::StateManager()
{
    m_modes[0] = new IdleState();
    m_modes[1] = new DisinfectionState();
    m_modes[2] = new ChargeState();
    m_modes[3] = new FaultState();

    m_currentState = m_modes[0];
    m_lastState = m_currentState;
}

StateManager::~StateManager()
{
    delete m_modes[0];
    delete m_modes[1];
    delete m_modes[2];
    delete m_modes[3];
}

void StateManager::changeState(StateBase *state)
{
    // 如果新的状态与当前状态不同，则改变状态
    if(state->m_state != m_currentState->m_state){
        m_lastState->exit();
        m_lastState = m_currentState;
        m_currentState = m_modes[state->m_state - 1];
        std::cout << "State: " << state->m_state << std::endl;
        m_currentState->handle();
    }
}

StateBase* StateManager::getCurrentState()
{
    return m_currentState;
}

StateBase* StateManager::getLastState()
{
    return m_lastState;
}
