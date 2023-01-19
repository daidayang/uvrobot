#ifndef _ODOM_DRIVER_HPP_
#define _ODOM_DRIVER_HPP_

#include <ros/ros.h>

#include <string>
#include <iostream>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <std_msgs/String.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/TimeReference.h>
#include <cstdlib>

#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <string>
#include <cstring>
extern "C"{
  #include <unistd.h>
  #include <errno.h>
}

class OdometryDriverRefactor
{
public:
    OdometryDriverRefactor();
    ~OdometryDriverRefactor();

    void init();
    void run();
    void cmdVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void cameraOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void laserReceivedCallback(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void getClickPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    void poseDataCallback(const std_msgs::String::ConstPtr &msg);
    void publishOdometry(std::vector<std::string> &odom_data);
    void publishOdometry(std::vector<double> &odom_data);

    std::vector<std::string> split(const std::string& str, char delim);
    int accessCheck(std::string portname);     // 检查是否拥有硬件的访问权限
    
    std::vector<std::string> explode(const std::string& text, const std::string& separators);
    void changePulseToOdomPose(std::vector<std::string> &pulse_list,std::vector<double> &odometry_data);
public:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    ros::Publisher m_odom_pub;
    ros::Publisher m_cmd_vel_pub;
    ros::Publisher m_laser_data_pub;
    ros::Publisher m_twist_cmd_pub;
    
    ros::Subscriber sub_pose;

    ros::Subscriber m_cmd_vel;
    ros::Subscriber m_pose_data;

    ros::Subscriber m_camera_odom_data;
    ros::Subscriber m_laser_data;

    std::string m_port;
    std::string m_frame_id;
    int m_baudrate;
    double m_wheel_gauge;//轮间距
    serial::Serial m_serial; //声明串口对象 
    bool m_open_serial;
    ros::Time tf_time;



    boost::asio::io_service m_ioserv;
    boost::asio::serial_port *m_sp;

    geometry_msgs::TransformStamped m_trans;
    tf::TransformBroadcaster m_broadcaster;



    //cal odom pose
    double m_x;
    double m_y;
    double m_theta;
    double m_linear_v;
    double m_angular_v;

    double m_wheel_track;//两个轮间距
    double m_wheel_diameter;//轮子直径
    double m_circle_enc; //轮子每圈脉冲总数

    int m_loop_time;
    int m_current_pulse_right;
    int m_last_pulse_right;

    int m_current_pulse_left;
    int m_last_pulse_left;

    bool m_first_get_encode_count;

};

#endif  // _ODOM_DRIVER_HPP_