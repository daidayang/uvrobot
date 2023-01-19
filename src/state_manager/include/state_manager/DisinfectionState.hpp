#ifndef _DISINFECTION_STATE_HPP_
#define _DISINFECTION_STATE_HPP_

#include <state_manager/StateBase.hpp>

class DisinfectionState : public StateBase
{
public:
    DisinfectionState();
    ~DisinfectionState();

    void handle();
    void exit();
};

#endif // _DISINFECTION_STATE_HPP_
