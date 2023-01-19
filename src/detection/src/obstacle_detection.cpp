#include "obstacle_detection.hpp"

ObstacleDetector::ObstacleDetector() : m_private_nh("~")
{
    m_private_nh.getParam("debug_flag", m_debug_flag);
    m_private_nh.getParam("costmap_topic", m_costmap_topic_name);
    m_private_nh.getParam("basic_map", m_basic_map_path);
    m_private_nh.getParam("robot_pose_topic", m_robot_pose_topic_name);
    m_private_nh.getParam("uvrobot_radius", m_uvrobot_radius);
    m_private_nh.getParam("map_resolution", m_map_resolution);
    m_private_nh.getParam("origin_x", m_origin_x);
    m_private_nh.getParam("origin_y", m_origin_y);
    m_private_nh.getParam("map_w", m_map_w);
    m_private_nh.getParam("map_h", m_map_h);

    if(m_debug_flag){
        std::cout << "costmap topic name: " << m_costmap_topic_name << std::endl;
        std::cout << "basic map path: " << m_basic_map_path << std::endl;
        std::cout << "robot pose topic: " << m_robot_pose_topic_name << std::endl;
        std::cout << "uvrobot radius: " << m_uvrobot_radius << std::endl;
        std::cout << "map resolution: " << m_map_resolution << std::endl;
        std::cout << "origin x: " << m_origin_x << std::endl;
        std::cout << "origin y: " << m_origin_y << std::endl;
    }

    m_sub_costmap = m_nh.subscribe(m_robot_pose_topic_name, 10, &ObstacleDetector::sub_robot_pose_callback, this);
    // m_pub_detection_res = m_nh.advertise<std_msgs::String>(m_detection_res_topic_name, 10);
    m_pub_detection_res = m_nh.advertise<obstacle_detection::obstacles>(m_detection_res_topic_name, 10);
    m_basic_map = cv::imread(m_basic_map_path, cv::IMREAD_GRAYSCALE);
}

ObstacleDetector::~ObstacleDetector()
{
    ros::shutdown();
}

void ObstacleDetector::sub_robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_ptr)
{
    // 获取车辆的附近范围，从图中扣取感兴趣区域
    nav_msgs::OccupancyGridConstPtr costmap_global;
    costmap_global = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(m_costmap_topic_name);
    float c_x_in_map = pose_ptr->pose.position.x; 
    float c_y_in_map = pose_ptr->pose.position.y;
    std::cout << "X : " << c_x_in_map <<std::endl;
    std::cout << "Y : " << c_y_in_map << std::endl;
    // 单位为米
    float roi_x_in_map = c_x_in_map - m_uvrobot_radius; 
    float roi_y_in_map = c_y_in_map - m_uvrobot_radius;
    // 单位为像素
    float roi_x_in_pic = abs((m_origin_x - c_x_in_map) / m_map_resolution);
    float roi_y_in_pic = m_map_h - abs((m_origin_y - c_y_in_map) / m_map_resolution);
    // float roi_x_in_pic = roi_x_in_map / m_map_resolution; roi_x_in_pic = abs(roi_x_in_pic);
    // float roi_y_in_pic = roi_y_in_map / m_map_resolution; roi_y_in_pic = costmap_global->info.height - abs(roi_y_in_pic);
    float roi_w = 2 * m_uvrobot_radius / m_map_resolution;
    float roi_h = 2 * m_uvrobot_radius / m_map_resolution;
    if(roi_x_in_pic < 0) roi_x_in_pic = 0;
    if(roi_y_in_pic < 0) roi_y_in_pic = 0;
    if(roi_x_in_pic + roi_w > costmap_global->info.width - 1) 
        roi_x_in_pic = costmap_global->info.width - roi_w - 1;
    if(roi_y_in_pic + roi_h > costmap_global->info.height - 1) 
        roi_y_in_pic = costmap_global->info.height - roi_h - 1;

    // std::cout << "cm w: " << costmap_global->info.width << std::endl;
    // std::cout << "cm h: " << costmap_global->info.height << std::endl;
    cv::Mat realtime_costmap(costmap_global->info.height, costmap_global->info.width, CV_8UC1, (void *)costmap_global->data.data());
    // for(int i = 0; i < realtime_costmap.rows; i++){
    //     for(int j = 0; j < realtime_costmap.cols; j++){
    //         if(realtime_costmap.at<uchar>(i, j) > 96){
    //             realtime_costmap.at<uchar>(i, j) = 255;
    //         }
    //     }
    // }
    cv::flip(realtime_costmap, realtime_costmap, 0);
    // cv::imwrite("/home/lor/costmap.jpg", realtime_costmap);
    // std::cout << "Pix: " << realtime_costmap.at<uchar>(84, 68) << std::endl;
    // std::cout << "img w: " << realtime_costmap.cols << std::endl;
    // std::cout << "img h: " << realtime_costmap.rows << std::endl;
    // std::cout << "roi x in pic: " << roi_x_in_pic << std::endl;
    // std::cout << "roi y in pic: " << roi_y_in_pic << std::endl;
    // std::cout << "roi w in pic: " << roi_w << std::endl;
    // std::cout << "roi h in pic: " << roi_h << std::endl;
    // std::cout << "b map w: " << m_basic_map.cols << std::endl;
    // std::cout << "b map h: " << m_basic_map.rows << std::endl;
    // std::cout << "63" << std::endl;
    cv::Mat roi_basicmap = m_basic_map(cv::Rect(int(roi_x_in_pic), int(roi_y_in_pic), int(roi_w), int(roi_h)));
    cv::Mat roi_costmap = realtime_costmap(cv::Rect(int(roi_x_in_pic), int(roi_y_in_pic), int(roi_w), int(roi_h))); 
    cv::Mat res(int(roi_w), int(roi_h), CV_8UC1);
    // cv::circle(realtime_costmap, cv::Point(roi_x_in_pic, roi_y_in_pic), 2, cv::Scalar(255, 225, 20));
    // cv::imwrite("/home/lor/m_basic_map_pos.jpg", realtime_costmap);
    cv::imwrite("/home/lor/roi_basicmap.jpg", roi_basicmap);
    cv::imwrite("/home/lor/roi_costmap.jpg", roi_costmap);
    // std::cout << "69" << std::endl;
    // std::cout << "roi b w: " << roi_basicmap.cols << "; roi b h: " << roi_basicmap.rows << std::endl;
    // std::cout << "roi c w: " << roi_costmap.cols << "; roi c h: " << roi_costmap.rows << std::endl;
    // cv::bitwise_xor(roi_basicmap, roi_costmap, res);

    // 像素值差超过128，则在结果图中置为亮
    for(int i = 0; i < res.rows; i++){
        for(int j = 0; j < res.cols; j++){
            if(abs(roi_basicmap.at<uchar>(i, j) - roi_costmap.at<uchar>(i, j)) > 128){
                res.at<uchar>(i, j) = 255;
            }
            else{
                res.at<uchar>(i, j) = 0;
            }
        }
    }
    cv::imwrite("/home/lor/res.jpg", res);
    // int val = cv::countNonZero(res);
    // std::cout << "value : " << val << std::endl;
    // float threshold = 50;

    // 获取障碍物轮廓信息
    std::vector< std::vector<cv::Point> > contours;
    cv::findContours(res, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    obstacle_detection::obstacles obs;
    for(int i = 0; i < contours.size(); i++){
        obstacle_detection::obstacle ob;
        for(int j = 0; j < contours[i].size(); j++){
            contours[i][j].x += roi_x_in_pic;
            contours[i][j].y += roi_y_in_pic;
            geometry_msgs::Point p;
            p.x = m_origin_x + (contours[i][j].x * m_map_resolution);
            p.y = m_origin_y + ((m_map_h - contours[i][j].y) * m_map_resolution);
            p.z = 0;
            ob.contours.push_back(p);
        }
        obs.obstacles.push_back(ob);
    }

    // cv::cvtColor(m_basic_map, m_basic_map, cv::COLOR_GRAY2BGR);
    if(m_debug_flag){
        std::cout << "Number of contour: " << contours.size() << std::endl;
        for(int i = 0; i < contours.size(); i++){
            cv::drawContours(m_basic_map, contours, i, cv::Scalar(100, 50, 100));
            std::cout << "Contour Area: " << cv::contourArea(contours[i]) << std::endl;
        } 
    }
    cv::imwrite("/home/lor/contours_res.jpg", m_basic_map);
    
    // std_msgs::String msg;
    // if(val > threshold){
    //     msg.data = "Warning";
    // }
    // else{
    //     msg.data = "Clear";
    // }
    // m_pub_detection_res.publish(msg);
}

void ObstacleDetector::run()
{
    ros::spin();
}