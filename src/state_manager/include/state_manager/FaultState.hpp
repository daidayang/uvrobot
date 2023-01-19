#ifndef _FAULT_STATE_HPP_
#define _FAULT_STATE_HPP_

#include <state_manager/StateBase.hpp>

class FaultState : public StateBase
{
public:
    FaultState();
    ~FaultState();

    void handle();
    void exit();
};

#endif // _FAULT_STATE_HPP_
