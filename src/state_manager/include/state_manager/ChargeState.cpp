#include <state_manager/ChargeState.hpp>

ChargeState::ChargeState()
{
    m_state = ST_CHARGING;
}

ChargeState::~ChargeState()
{

}

void ChargeState::handle()
{
    // 处理充电状态进入时的节点操作
}

void ChargeState::exit()
{
    // 处理充电状态退出时的节点操作
}
