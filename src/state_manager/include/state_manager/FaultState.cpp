#include <state_manager/FaultState.hpp>

FaultState::FaultState()
{
        m_state = ST_FAULT;
}

FaultState::~FaultState()
{

}

void FaultState::handle()
{
    // 处理故障状态进入时的节点处理
}

void FaultState::exit()
{
    // 处理故障状态退出时的节点操作
}
