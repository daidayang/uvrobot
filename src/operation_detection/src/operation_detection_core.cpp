#include "operation_detection_core.hpp"

OperationDetector::OperationDetector() : m_private_nh("~"), m_fast_op_det_flag(false)
{
    m_private_nh.getParam("debug_flag", m_debug_flag);
    m_private_nh.getParam("cmd_vel_sub_topic_name", m_cmd_vel_sub_topic_name);
    m_private_nh.getParam("robot_pose_topic_name", m_robot_pose_topic_name);
    m_private_nh.getParam("op_det_pub_topic_name", m_op_det_pub_topic_name);
    m_private_nh.getParam("threshold_decision", m_threshold_decision);
    m_private_nh.getParam("span_time", m_span_time);
    m_private_nh.getParam("fast_op_det_pub_topic_name", m_fast_op_det_pub_topic_name);

    if(m_debug_flag){
        std::cout << "cmd vel subscribe topic: " << m_cmd_vel_sub_topic_name << std::endl;
        std::cout << "robot pose topic name: " << m_robot_pose_topic_name << std::endl;
        std::cout << "operation detection result publish topic: " << m_op_det_pub_topic_name << std::endl;
        std::cout << "threshold decision: " << m_threshold_decision << std::endl;
        std::cout << "span time: " << m_span_time << std::endl;
        std::cout << "fast operation detection result topic: " << m_fast_op_det_pub_topic_name << std::endl;
    }

    m_current_time = 0;
    m_history_time = 0;
    m_first_run_flag = 1;

    m_current_ropose.x = 0;
    m_current_ropose.y = 0;
    m_current_ropose.z = 0;
    m_history_ropose.x = 0;
    m_history_ropose.y = 0;
    m_history_ropose.z = 0;

    m_sub_cmd_vel = m_nh.subscribe(m_cmd_vel_sub_topic_name, 10, &OperationDetector::cmd_vel_sub_callback, this);
    m_pub_op_det = m_nh.advertise<std_msgs::String>(m_op_det_pub_topic_name, 10, true);
    m_pub_fast_op_det = m_nh.advertise<std_msgs::String>(m_fast_op_det_pub_topic_name, 10);
    m_timer = m_nh.createTimer(ros::Duration(m_span_time), &OperationDetector::timer_callback, this);
}

OperationDetector::~OperationDetector()
{
    ros::shutdown;
}   

int linear_flag = 0;    // 当cmd_vel发过来的是控制线速度的命令时
void OperationDetector::cmd_vel_sub_callback(const geometry_msgs::Twist::ConstPtr &cmd_ptr)
{
    m_current_time = ros::Time::now().toSec();

    geometry_msgs::PoseStampedConstPtr robot_pose_ptr;
    robot_pose_ptr = ros::topic::waitForMessage<geometry_msgs::PoseStamped>(m_robot_pose_topic_name);
    
    m_current_ropose.x = robot_pose_ptr->pose.position.x;
    m_current_ropose.y = robot_pose_ptr->pose.position.y;
    m_current_ropose.z = robot_pose_ptr->pose.position.z;

    if(cmd_ptr->linear.x != 0 || cmd_ptr->linear.y != 0){
        linear_flag = 1;
    }

    if(m_first_run_flag){
        m_history_time = m_current_time;
        m_history_ropose = m_current_ropose;
        m_first_run_flag = 0;
    }

    if(m_fast_op_det_flag){
        std_msgs::String msg;
        msg.data = "Fault";
        m_pub_fast_op_det.publish(msg);
    }

    // if(m_current_time - m_history_time > 10){
    //     if(pow((m_current_ropose.x - m_history_ropose.x), 2) + pow((m_current_ropose.y - m_history_ropose.y), 2)){
    //         std::cout << "Robot NOT response to the control command, please check" << std::endl;
    //     }
    // }
}

void OperationDetector::timer_callback(const ros::TimerEvent &event)
{
    if(linear_flag)
    {
        std::cout << "71 " << std::endl;
        if( (pow((m_current_ropose.x - m_history_ropose.x), 2) + pow((m_current_ropose.y - m_history_ropose.y), 2)) < pow(m_threshold_decision, 2) ){
            std::cout << "Robot NOT response to the control command, please check" << std::endl;
            std_msgs::String msg;
            msg.data = "Not Running";
            m_pub_op_det.publish(msg);
            m_fast_op_det_flag = true;
        }
        else{
            m_history_ropose = m_current_ropose;
            m_history_time = m_current_time;
            std_msgs::String msg;
            msg.data = "Running";
            m_pub_op_det.publish(msg);
            m_fast_op_det_flag = false;
        }

        linear_flag = 0;
    }

}

void OperationDetector::run()
{
    ros::spin();
}