#include "state_manager_core.hpp"

StateManagerCore::StateManagerCore()
    : m_private_nh("~")
    , m_state_flag(false)
    , m_operation_flag(false)
{
    m_private_nh.getParam("debug_flag", m_debug_flag);
    m_private_nh.getParam("battery_topic_name", m_battery_state_topic);
    m_private_nh.getParam("task_topic_name", m_task_state_topic);
    m_private_nh.getParam("critical_error_topic_name", m_critical_error_topic);
    m_private_nh.getParam("uvrobot_state_topic", m_uvrobot_state_topic);
    m_private_nh.getParam("uvrobot_operation_topic", m_uvrobot_operation_topic);
    m_private_nh.getParam("navigation_switch_topic_name", m_navigation_switch_topic);

    if(m_debug_flag){
        std::cout << "Battery state topic: " << m_battery_state_topic << std::endl;
        std::cout << "Task topic: " << m_task_state_topic << std::endl;
        std::cout << "Critical error topic: " << m_critical_error_topic << std::endl;
        std::cout << "UVRobot state topic: " << m_uvrobot_state_topic << std::endl;
        std::cout << "Navigation switch topic: " << m_navigation_switch_topic << std::endl;
    }

    // m_timer1 = m_nh.createTimer(ros::Duration(1), &StateManagerCore::timer_cb1, this);
    // m_timer2 = m_nh.createTimer(ros::Duration(1), &StateManagerCore::timer_cb2, this);
    // m_timer3 = m_nh.createTimer(ros::Duration(1), &StateManagerCore::timer_cb3, this);
    // m_timer4 = m_nh.createTimer(ros::Duration(1), &StateManagerCore::timer_cb4, this);
    // m_timer5 = m_nh.createTimer(ros::Duration(1), &StateManagerCore::timer_cb5, this);
    // m_timer6 = m_nh.createTimer(ros::Duration(1), &StateManagerCore::timer_cb6, this);
    // m_timer7 = m_nh.createTimer(ros::Duration(1), &StateManagerCore::timer_cb7, this);
    // m_timer8 = m_nh.createTimer(ros::Duration(1), &StateManagerCore::timer_cb8, this);
    m_pub_uvrobot_state = m_nh.advertise<std_msgs::Int32>(m_uvrobot_state_topic, 10);

    m_sub_uvrobot_state = m_nh.subscribe(m_uvrobot_state_topic, 10, &StateManagerCore::uvrobot_state_sub_callback, this);
    m_sub_uvrobot_operation = m_nh.subscribe(m_uvrobot_operation_topic, 10, &StateManagerCore::uvrobot_operation_sub_callback, this);


    // 启动navigation的工作
    m_pub_navigation_switch = m_nh.advertise<std_msgs::Bool>(m_navigation_switch_topic, 10);
    std_msgs::Bool msg;
    msg.data = true;
    m_pub_navigation_switch.publish(msg);
}

StateManagerCore::~StateManagerCore()
{
    ros::shutdown();
}

void StateManagerCore::run()
{
    ros::spin();
}

void StateManagerCore::uvrobot_state_sub_callback(const std_msgs::Int32 &msg)
{
    std::cout << "Current state: " << msg.data << std::endl;
    // 故障状态拥有最高的优先级
    if(!m_operation_flag){
        if(msg.data == 1){
            DisinfectionState *ds;
            ds = new DisinfectionState();
            m_state_manager.changeState(ds);
        }
        if(msg.data == 2){
            ChargeState *cs;
            cs = new ChargeState();
            m_state_manager.changeState(cs);
        }   
        if(msg.data == 3){
            IdleState *is;
            is = new IdleState();
            m_state_manager.changeState(is);
        }
    }
}

void StateManagerCore::uvrobot_operation_sub_callback(const std_msgs::String::ConstPtr &msg_ptr)
{
    if(msg_ptr->data == "Running"){
        m_operation_flag = false;
        std_msgs::Bool start_msg;
        start_msg.data = true;
        m_pub_navigation_switch.publish(start_msg);
    }
    
    if(msg_ptr->data == "Not Running"){
        m_operation_flag = true;
        FaultState *fs;
        fs = new FaultState();
        m_state_manager.changeState(fs);
        std_msgs::String msg;
        msg.data = "Fault";
        m_pub_uvrobot_state.publish(msg);

        std_msgs::Bool start_msg;
        start_msg.data = false;
        m_pub_navigation_switch.publish(start_msg);
    }
}

// void StateManagerCore::timer_cb1(const ros::TimerEvent &evt)
// {
//     std::cout << "before 1" << std::endl;
//     std_msgs::String::ConstPtr msg = ros::topic::waitForMessage<std_msgs::String>("test");
//     std::cout << "CB 1 " << std::endl;
// }

// void StateManagerCore::timer_cb2(const ros::TimerEvent &evt)
// {
//     std::cout << "CB 2 " << std::endl;
// }
// void StateManagerCore::timer_cb3(const ros::TimerEvent &evt)
// {
//     std::cout << "CB 3 " << std::endl;
// }
// void StateManagerCore::timer_cb4(const ros::TimerEvent &evt)
// {
//     std::cout << "CB 4 " << std::endl;
// }

// void StateManagerCore::timer_cb5(const ros::TimerEvent &evt)
// {
//     std::cout << "CB 5 " << std::endl;
// }

// void StateManagerCore::timer_cb6(const ros::TimerEvent &evt)
// {
//     std::cout << "CB 6 " << std::endl;
// }
// void StateManagerCore::timer_cb7(const ros::TimerEvent &evt)
// {
//     std::cout << "CB 7 " << std::endl;
// }
// void StateManagerCore::timer_cb8(const ros::TimerEvent &evt)
// {
//     std::cout << "CB 8 " << std::endl;
// }
