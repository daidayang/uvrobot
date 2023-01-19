#ifndef _STATE_BASE_HPP_
#define _STATE_BASE_HPP_

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>

// enum States
// {
//     ST_IDLE = 0,
//     ST_DISINFECTION,
//     ST_CHARGE,
//     ST_ERROR
// };

enum States
{
    ST_WORKING  = 1,    
    ST_CHARGING = 2,    
    ST_IDLE     = 3,    
    ST_FAULT    = 4
};

class StateManager;

class StateBase
{
public:
    StateBase(){}

    virtual ~StateBase(){}

    virtual void handle() = 0;  // 处理状态进入的操作
    virtual void exit() = 0;    // 状态退出时必要的善后操作

public:
    States m_state;
};

#endif // _STATE_BASE_HPP_
