#ifndef _OBSTACLE_DETECTION_HPP_
#define _OBSTACLE_DETECTION_HPP_

#include <ros/ros.h>
#include <ros/topic.h>
#include <std_msgs/String.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_layer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <obstacle_detection/obstacle.h>
#include <obstacle_detection/obstacles.h>

#include <iostream>
#include <vector>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class ObstacleDetector
{
public:
    ObstacleDetector();
    ~ObstacleDetector();

    void run();
    // void init();    // 初始化机器人的外轮廓以及
    void sub_robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_ptr);

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_private_nh;
    ros::Subscriber m_sub_costmap;
    ros::Publisher m_pub_detection_res;

    int m_debug_flag;
    std::string m_costmap_topic_name;
    std::string m_detection_res_topic_name;
    std::string m_robot_pose_topic_name;
    std::string m_basic_map_path;
    cv::Mat m_basic_map;

    float m_uvrobot_radius;
    float m_map_resolution;
    float m_origin_x;
    float m_origin_y;
    float m_map_w;
    float m_map_h;
};

#endif //_OBSTALCE_DETECTION_HPP_