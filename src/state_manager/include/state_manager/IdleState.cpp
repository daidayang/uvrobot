#include <state_manager/IdleState.hpp>

IdleState::IdleState()
{
    m_state = ST_IDLE;
}

IdleState::~IdleState()
{

}

void IdleState::handle()
{
    // 处理状态进入时的节点操作
}

void IdleState::exit()

{
    // 处理状态退出时的节点操作
}
