#ifndef _CHARGE_STATE_HPP_
#define _CHARGE_STATE_HPP_

#include <state_manager/StateBase.hpp>

class ChargeState : public StateBase
{
public:
    ChargeState();
    ~ChargeState();

    void handle();
    void exit();
};

#endif // _CHARGE_STATE_HPP_
