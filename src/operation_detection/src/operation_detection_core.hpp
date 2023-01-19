#ifndef _OPERATION_DETECTION_CORE_HPP_
#define _OPERATION_DETECTION_CORE_HPP_

#include <ros/ros.h>
#include <ros/topic.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <vector>
#include <cmath>

class OperationDetector
{
public:
    OperationDetector();
    ~OperationDetector();

    void run();
    void cmd_vel_sub_callback(const geometry_msgs::Twist::ConstPtr &cmd_ptr);
    void timer_callback(const ros::TimerEvent &event);

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_private_nh;
    ros::Subscriber m_sub_cmd_vel;
    ros::Publisher m_pub_op_det;
    ros::Publisher m_pub_fast_op_det;
    ros::Timer m_timer;

    float m_current_time;   // 记录当前时刻
    float m_history_time;   // 记录历史时刻
    int m_first_run_flag;
    float m_threshold_decision;
    float m_span_time;      // 检测车辆不响应的延迟时间

    geometry_msgs::Point m_history_ropose;  // 记录历史UVRobot位置
    geometry_msgs::Point m_current_ropose;  // 记录当前UVRobot位置

    std::string m_cmd_vel_sub_topic_name;
    std::string m_robot_pose_topic_name;
    std::string m_op_det_pub_topic_name;
    std::string m_fast_op_det_pub_topic_name;

    int m_debug_flag;
    bool m_fast_op_det_flag;
};

#endif