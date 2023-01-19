#include <state_manager/DisinfectionState.hpp>

DisinfectionState::DisinfectionState()
{
    m_state = ST_WORKING;
}

DisinfectionState::~DisinfectionState()
{

}

void DisinfectionState::handle()
{
    // 处理消毒巡逻状态进入时的节点操作
}

void DisinfectionState::exit()
{
    // 处理消毒巡逻状态退出时的节点操作
}
