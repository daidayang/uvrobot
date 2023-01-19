#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base/TaskData.h>

#include <iostream>
#include <vector>

class Repeater
{
public:
    Repeater() : m_id_count(0), m_private_nh("~")
    {
        m_private_nh.getParam("debug_flag", m_debug_flag);
        m_private_nh.getParam("pub_topic_name", m_pub_topic_name);
        m_private_nh.getParam("time_interval", m_time_interval);
        if(m_debug_flag){
            std::cout << "publish topic name: " << m_pub_topic_name << std::endl;
            std::cout << "time interval: " << m_time_interval << " min(s)" << std::endl;
        }

        m_pub = m_nh.advertise<move_base::TaskData>(m_pub_topic_name, 10);
        m_timer = m_nh.createTimer(ros::Duration(m_time_interval * 60), &Repeater::timer_callback, this);
    }
    
    ~Repeater()
    {
        ros::shutdown();
    }

    void run()
    {
        ros::spin();
    }

    void timer_callback(const ros::TimerEvent &eve)
    {
        std::cout << ss_time_format() << std::endl;
        m_id_count++;

        move_base::TaskData task_msg;
        task_msg.id = m_id_count;
        task_msg.task_id = m_id_count * 2 + 1;
        task_msg.room_id = std::to_string(m_id_count*2+ 1);
        // 填充 goal_list
        for(int i = 0; i < 21; i++){
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = -0.65;
            pose.pose.position.y = -0.707;
            pose.pose.orientation.w = 0.978;
            task_msg.goal_list.push_back(pose);

            pose.pose.position.x = 0.101;
            pose.pose.position.y = -1.14;
            pose.pose.orientation.w = -0.185;
            task_msg.goal_list.push_back(pose);

            pose.pose.position.x = 0.116;
            pose.pose.position.y = -0.28;
            pose.pose.orientation.w = 0.696;
            task_msg.goal_list.push_back(pose);
        }
        // 填充duration
        for(int i = 0; i < 63; i++){
            task_msg.goal_duration_list.push_back(5);
        }
        for(int i = 0; i < 63; i++){
            task_msg.command_previous.push_back("cx#cx");
            task_msg.command_back.push_back("c1#c7");
        }

        task_msg.type = 0;
        task_msg.states = "todo";
        task_msg.exec_time = ss_time_format();
        task_msg.add_or_delete = 1;
        task_msg.resend_task = false;

        std::cout << "76" << std::endl;
        m_pub.publish(task_msg);
    }

    std::string ss_time_format()
    {
        char _time[16];
        time_t rawtime;
        time(&rawtime);
        struct tm *cur_time = localtime(&rawtime);

        std::stringstream ss;

        ss << cur_time->tm_year + 1900 << "-";
        ss << std::setfill('0') << std::setw(2) << cur_time->tm_mon + 1 << "-";
        ss << std::setfill('0') << std::setw(2) << cur_time->tm_mday << " ";
        ss << std::setfill('0') << std::setw(2) << cur_time->tm_hour << ":";
        ss << std::setfill('0') << std::setw(2) << cur_time->tm_min << ":";
        ss << std::setfill('0') << std::setw(2) << cur_time->tm_sec;

        std::string ss_time = ss.str();

        return ss_time;
    }

    ros::NodeHandle m_nh;
    ros::NodeHandle m_private_nh;
    ros::Publisher m_pub;
    std::string m_pub_topic_name;

    // move_base::TaskData m_taskdata;
    ros::Timer m_timer;
    int m_id_count;
    float m_time_interval;    // in minute

    int m_debug_flag;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_repeat");

    Repeater repeater;
    repeater.run();

    return 0;
}