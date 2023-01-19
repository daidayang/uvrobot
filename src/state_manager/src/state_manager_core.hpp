#ifndef _STATE_MANAGER_CORE_HPP
#define _STATE_MANAGER_CORE_HPP

#include <ros/ros.h>
#include <ros/topic.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <state_manager/StateManager.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <vector>

class StateManagerCore
{
public:
    StateManagerCore();
    ~StateManagerCore();

    void run();

    void uvrobot_state_sub_callback(const std_msgs::Int32 &msg);
    void uvrobot_operation_sub_callback(const std_msgs::String::ConstPtr &msg_ptr);

    // void timer_cb1(const ros::TimerEvent &evt);
    // void timer_cb2(const ros::TimerEvent &evt);
    // void timer_cb3(const ros::TimerEvent &evt);
    // void timer_cb4(const ros::TimerEvent &evt);
    // void timer_cb5(const ros::TimerEvent &evt);
    // void timer_cb6(const ros::TimerEvent &evt);
    // void timer_cb7(const ros::TimerEvent &evt);
    // void timer_cb8(const ros::TimerEvent &evt);

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_private_nh;

    ros::Publisher m_pub_uvrobot_state;
    ros::Publisher m_pub_navigation_switch;
    ros::Subscriber m_sub_uvrobot_state;
    ros::Subscriber m_sub_uvrobot_operation;

    // ros::Timer m_timer1;
    // ros::Timer m_timer2;
    // ros::Timer m_timer3;
    // ros::Timer m_timer4;
    // ros::Timer m_timer5;
    // ros::Timer m_timer6;
    // ros::Timer m_timer7;
    // ros::Timer m_timer8;

    StateManager m_state_manager;

    std::string m_battery_state_topic;
    std::string m_task_state_topic;
    std::string m_critical_error_topic;
    std::string m_uvrobot_state_topic;
    std::string m_uvrobot_operation_topic;
    std::string m_navigation_switch_topic;
    int m_debug_flag;

    bool m_state_flag;
    bool m_operation_flag;
};

#endif // _STATE_MANAGER_CORE_HPP
