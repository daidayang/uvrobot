#ifndef _WSSERVER_H_
#define _WSSERVER_H_

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <sstream>
#include <string>
#include <cstdlib>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>
// #include <opencv2/videostab/videostab.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base/TaskData.h>

#include <json.hpp>
#include <curl/curl.h>

extern "C"{
#include <unistd.h>
}

using json = nlohmann::json;

typedef websocketpp::server<websocketpp::config::asio> server;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

// pull out the type of messages sent by our config
typedef server::message_ptr message_ptr;

static const std::string base64_chars = 
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789+/";

class WSServer
{
public:
    WSServer();
    ~WSServer();
    
    void run();
    void init();
    void on_open(server *s, websocketpp::connection_hdl hdl);
    void on_message(server *s, websocketpp::connection_hdl hdl, message_ptr msg);
    void position_thread();
    void patrol_status_thread();
    // void test_patrol_status_thread();
    // void position_sub_callback(const std_msgs::String::ConstPtr msg_ptr);
    void position_sub_callback(const geometry_msgs::PoseStamped::ConstPtr msg_ptr);
    void patrol_status_sub_callback(const move_base::TaskData::ConstPtr msg_ptr);
    // void test_patrol_status_sub_cb(const std_msgs::String::ConstPtr msg_ptr);

    // static inline bool is_base64(unsigned char c);
    std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len);
    std::string base64_decode(std::string const& encoded_string);
    std::vector<std::string> explode(const std::string& text, const std::string& separators);
    std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method);

private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_private_nh;
    ros::Subscriber m_position_subscriber;
    ros::Subscriber m_patrol_status_sub;
    ros::Publisher m_remote_control_cmd_pub;
    ros::Publisher m_clean_task_pub;

    std::string m_topic_name;
    std::string m_remote_cmd_topic_name;    
    std::string m_patrol_status_topic_name;
    std::string m_clean_task_topic_name;
    int m_debug_flag;
    int m_ws_port;

    websocketpp::connection_hdl m_hdl;
    server m_server;

    cv::VideoCapture m_cap;
    std::string m_report_status_json;
    std::string m_report_status_url;
    int m_cam_type;
    int m_cam_id;   // for USB camera
    
    int m_count;
    float m_map_width;
    float m_map_height;
    float m_map_origin_x;
    float m_map_origin_y;
    float m_map_resolution;
};

#endif // _WSSERVER_H_
