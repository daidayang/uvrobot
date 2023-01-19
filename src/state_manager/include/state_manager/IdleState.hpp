#ifndef _IDLE_STATE_HPP_
#define _IDLE_STATE_HPP_

#include <state_manager/StateBase.hpp>

class IdleState : public StateBase
{
public:
    IdleState();
    ~IdleState();
    void handle();
    void exit();
};

#endif // _IDLE_STATE_HPP_
