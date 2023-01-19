#include "wsserver_core.hpp"

WSServer::WSServer() : m_private_nh("~"), m_count(0)
{
    m_private_nh.getParam("ws_port", m_ws_port);
    m_private_nh.getParam("debug_flag", m_debug_flag);
    m_private_nh.getParam("robot_position_topic_name", m_topic_name);
    m_private_nh.getParam("remote_control_topic_name", m_remote_cmd_topic_name);
    m_private_nh.getParam("patrol_status_topic_name", m_patrol_status_topic_name);
    m_private_nh.getParam("report_status_url", m_report_status_url);
    m_private_nh.getParam("clean_task_topic_name", m_clean_task_topic_name);
    m_private_nh.getParam("map_width", m_map_width);
    m_private_nh.getParam("map_height", m_map_height);
    m_private_nh.getParam("map_origin_x", m_map_origin_x);
    m_private_nh.getParam("map_origin_y", m_map_origin_y);
    m_private_nh.getParam("map_resolution", m_map_resolution);
    m_private_nh.getParam("csi_or_usb", m_cam_type);
    m_private_nh.getParam("cam_id", m_cam_id);

    // m_ws_port = 9002;
    // m_debug_flag = 1;
    // m_topic_name = "robot_pose";
    // m_remote_cmd_topic_name = "cmd_vel";
    if(m_debug_flag){
        std::cout << "websocket port: " << m_ws_port << std::endl;
        std::cout << "robot position topic: " << m_topic_name << std::endl;
        std::cout << "remote control topic: " << m_remote_cmd_topic_name << std::endl;
        std::cout << "patrol status topic name: " << m_patrol_status_topic_name << std::endl;
        std::cout << "report status URL: " << m_report_status_url << std::endl;
        std::cout << "map width: " << m_map_width << std::endl;
        std::cout << "map height: " << m_map_height << std::endl;
        std::cout << "map origin x: " << m_map_origin_x << std::endl;
        std::cout << "map origin y: " << m_map_origin_y << std::endl;
        std::cout << "map resolution: " << m_map_resolution << std::endl;
        std::cout << "camera type: " << (m_cam_type ? "USB" : "CSI") << std::endl;
        std::cout << "USB camera id: " << m_cam_id << std::endl;
    }

    // m_remote_control_cmd_pub = m_nh.advertise<std_msgs::String>(m_remote_cmd_topic_name, 10);
    m_clean_task_pub = m_nh.advertise<move_base::TaskData>(m_clean_task_topic_name, 10);

    // init();
}

WSServer::~WSServer()
{
    ros::shutdown();
}

void WSServer::init()
{

}

std::string WSServer::gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

std::vector<std::string> WSServer::explode(const std::string& text, const std::string& separators)
{
    std::vector<std::string> words;
    size_t n     = text.length ();
    size_t start = 0;
    size_t stop = text.find_first_of (separators);
    if (stop > n) stop = n;

    while (start <= n)
    {
        words.push_back (text.substr (start, stop-start));
        start = stop+1;
        stop = text.find_first_of (separators, start);
        if (stop > n) stop = n;
    }
    return words;
}

// static inline bool WSServer::is_base64(unsigned char c)
// {
//     return (isalnum(c) || (c == '+') || (c == '/'));
// }

std::string WSServer::base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len) {
  std::string ret;
  int i = 0;
  int j = 0;
  unsigned char char_array_3[3];
  unsigned char char_array_4[4];
 
  while (in_len--) {
    char_array_3[i++] = *(bytes_to_encode++);
    if (i == 3) {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;
 
      for(i = 0; (i <4) ; i++)
        ret += base64_chars[char_array_4[i]];
      i = 0;
    }
  }
 
  if (i)
  {
    for(j = i; j < 3; j++)
      char_array_3[j] = '\0';
 
    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
    char_array_4[3] = char_array_3[2] & 0x3f;
 
    for (j = 0; (j < i + 1); j++)
      ret += base64_chars[char_array_4[j]];
 
    while((i++ < 3))
      ret += '=';
 
  }
 
  return ret;
}

void WSServer::position_thread()
{
    ros::MultiThreadedSpinner spinner(2);

    m_position_subscriber = m_nh.subscribe(m_topic_name, 10, &WSServer::position_sub_callback, this);
    
    spinner.spin();
}

void WSServer::patrol_status_thread()
{
    ros::MultiThreadedSpinner spinner(2);
    m_patrol_status_sub = m_nh.subscribe(m_patrol_status_topic_name, 10, &WSServer::patrol_status_sub_callback, this);

    spinner.spin();
}

// void WSServer::test_patrol_status_thread()
// {
//     ros::MultiThreadedSpinner spinner(2);
//     m_patrol_status_sub = m_nh.subscribe(m_patrol_status_topic_name, 10, &WSServer::test_patrol_status_sub_cb, this);

//     spinner.spin();
// }

void WSServer::position_sub_callback(const geometry_msgs::PoseStamped::ConstPtr msg_ptr)
{
    std::cout << msg_ptr->pose.position.x << std::endl;
    std::cout << msg_ptr->pose.position.y << std::endl;
    // 需要根据origin的正负进行一定的修改，先以负
    float x_pixel = abs((m_map_origin_x - msg_ptr->pose.position.x) / m_map_resolution);
    float y_pixel = m_map_height - abs((m_map_origin_y - msg_ptr->pose.position.y) / m_map_resolution);

    std::string position_json = "{";
    position_json += "\"command\":\"position\",";
    position_json += "\"x\":" + std::to_string(x_pixel) + ",";
    position_json += "\"y\":" + std::to_string(y_pixel);
    position_json += "}";
    std::cout << position_json << std::endl;

    m_server.send(m_hdl, position_json, websocketpp::frame::opcode::TEXT);

    // m_server.send(m_hdl, msg_ptr->data, websocketpp::frame::opcode::TEXT);
}

// void WSServer::test_patrol_status_sub_cb(const std_msgs::String::ConstPtr msg_ptr)
// {
//     m_report_status_json = "{";
//     m_report_status_json += "\"taskid\":" + std::to_string(101) + ",";
//     m_report_status_json += "\"report_status\":\"" + std::string("done") + "\",";
//     m_report_status_json += "\"resend_task\":" + std::to_string(0);
//     m_report_status_json += "}";

//     CURL *curl;
//     CURLcode retval;
//     retval = curl_global_init(CURL_GLOBAL_DEFAULT);
//     if( retval != CURLE_OK ){
//         std::cout << "curl_global_init() failed " << curl_easy_strerror(retval) << std::endl;
//         return;
//     }
//     curl = curl_easy_init();
//     if(curl){
//         struct curl_slist *header = NULL;
//         header = curl_slist_append(header, "Content-Type:application/json");
//         curl_easy_setopt(curl, CURLOPT_URL, m_report_status_url.c_str());
//         curl_easy_setopt(curl, CURLOPT_POST, 1L);
//         curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);

//         curl_easy_setopt(curl, CURLOPT_HTTPHEADER, header);
//         std::cout << "report status json: " << m_report_status_json << std::endl;
//         curl_easy_setopt(curl, CURLOPT_POSTFIELDS, m_report_status_json.c_str());
//         retval = curl_easy_perform(curl);
//         // std::cout << "Error:" << curl_easy_strerror(res) << std::endl;
//         if(retval != CURLE_OK){
//             std::cout << "return value: " << retval << std::endl;
//             std::cout << "curl_easy_perform() failed" << std::endl;
//             return;
//         }
//         curl_easy_cleanup(curl);
//     }
// }

// 通过http post方式向Web服务器返回当前巡逻的状态
void WSServer::patrol_status_sub_callback(const move_base::TaskData::ConstPtr msg_ptr)
{
    // std::cout << "patrol status subscriber: " << msg_ptr->data << std::endl;
    /*
     * code
     *      组装巡逻状态报告的json字符串
     * */
    m_report_status_json = "{";
    m_report_status_json += "\"taskid\":" + std::to_string(msg_ptr->task_id) + ",";
    m_report_status_json += "\"report_status\":\"" + msg_ptr->states + "\",";
    m_report_status_json += "\"resend_task\":" + std::to_string((msg_ptr->resend_task ? 1 : 0));
    m_report_status_json += "}";

    CURL *curl;
    CURLcode retval;
    retval = curl_global_init(CURL_GLOBAL_DEFAULT);
    if( retval != CURLE_OK ){
        std::cout << "curl_global_init() failed " << curl_easy_strerror(retval) << std::endl;
        return;
    }
    curl = curl_easy_init();
    if(curl){
        struct curl_slist *header = NULL;
        header = curl_slist_append(header, "Content-Type:application/json");
        curl_easy_setopt(curl, CURLOPT_URL, m_report_status_url.c_str());
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        // curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, header);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, m_report_status_json.c_str());
        retval = curl_easy_perform(curl);
        // std::cout << "Error:" << curl_easy_strerror(res) << std::endl;
        if(retval != CURLE_OK){
            std::cout << "curl_easy_perform() failed" << std::endl;
            return;
        }
        curl_easy_cleanup(curl);
    }
}

void WSServer::on_open(server *s, websocketpp::connection_hdl hdl)
{
    std::cout << "Already Open" << std::endl;
    m_hdl = hdl;
    try{
        std::thread t_position(&WSServer::position_thread, this);
        std::thread t_status(&WSServer::patrol_status_thread, this);

        t_position.detach();
        t_status.detach();
        // t.join();
    }
    catch(const std::exception &e){
        std::cerr << "Open: " << e.what() << std::endl;
    }
}

void WSServer::on_message(server *s, websocketpp::connection_hdl hdl, message_ptr msg)
{
    std::cout << "on_message called with hdl: " << hdl.lock().get()
              << " and message: " << msg->get_payload()
              << std::endl;

    std::cout << "***** " << msg->get_payload().substr(1, 22) << std::endl;

    if(msg->get_payload() == "command:video_start"){
        if(m_cam_type){
            // 使用USB相机
            m_cap.open(m_cam_id);
        }
        else{
            // 使用CSI相机（树莓派相机）
            std::string csi_cam_str = gstreamer_pipeline(1920, 1080, 640, 480, 30, 0);
            m_cap.open(csi_cam_str, cv::CAP_GSTREAMER);
        }
        if(!m_cap.isOpened()){
            std::cout << "Failed to open camera" << std::endl;
        }
        else{
            std::string video_rdy_json = "{\"command\":\"video_ready\"}";
            s->send(hdl, video_rdy_json, websocketpp::frame::opcode::TEXT);
        }
    }
    else if(msg->get_payload() == "command:video_tick"){
        cv::Mat img;
        m_cap >> img;
        std::vector<uchar> data_encode;
        cv::imencode(".jpg", img, data_encode);
        std::string img_encode_str(data_encode.begin(), data_encode.end());
        std::string img_base64 = base64_encode(reinterpret_cast<const unsigned char*>(img_encode_str.c_str()), img_encode_str.length());
        std::string img_json = "{";
        img_json += "\"command\": \"image\",";
        img_json += "\"payload\":\"" + img_base64 + "\"";
        img_json += "}";

        // s->send(hdl, img_base64, websocketpp::frame::opcode::TEXT);
        s->send(m_hdl, img_json, websocketpp::frame::opcode::TEXT);
    }
    else if(msg->get_payload() == "command:video_stop"){
        m_cap.release();
    }
    // 机器人方向控制
    else if(msg->get_payload() == "command:left" ||
            msg->get_payload() == "command:right" ||
            msg->get_payload() == "command:forward" ||
            msg->get_payload() == "command:backward" ||
            msg->get_payload() == "command:pause"){
        std::vector<std::string> parts = explode(msg->get_payload(), ":");
        std_msgs::String control_msg;
        /* 数据域赋值 */
        m_remote_control_cmd_pub.publish(control_msg);
    }
    // 机器人速度设置
    else if(msg->get_payload() == "command:velo_1" ||
            msg->get_payload() == "command:velo_2" || 
            msg->get_payload() == "command:velo_3"){
        std::vector<std::string> parts = explode(msg->get_payload(), ":");
        std_msgs::String control_msg;
        /* 数据域赋值 */
        m_remote_control_cmd_pub.publish(control_msg);
    }
    // Unused: 消毒任务写入完成，写文件的方式
    else if(msg->get_payload() == "clean_task_write"){
        std_msgs::String msg;
        msg.data = "done";
        m_clean_task_pub.publish(msg);
    }
    // 消毒任务通知，topic的方式
    else if(msg->get_payload().substr(1, 22) == "\"command\":\"clean_task\""){
        std::string raw_msg = msg->get_payload();
        json task_json = json::parse(raw_msg);
        std::cout << "***" << raw_msg << std::endl;
        std::cout << task_json["command"] << std::endl;
        std::cout << task_json["exec_time"] << std::endl;
        std::cout << task_json["task_id"] << std::endl;
        std::cout << task_json["room_id"] << std::endl;
        std::cout << task_json["position"] << std::endl;
        std::cout << task_json["duration"] << std::endl;
        std::cout << task_json["type"] << std::endl;
        std::cout << task_json["report_status"] << std::endl;

        move_base::TaskData task_msg;
        m_count++;
        task_msg.id = m_count;
        task_msg.task_id = task_json["task_id"];
        task_msg.room_id = task_json["room_id"];
        std::vector<std::string> points = explode(task_json["position"], ";");
        for(int i = 0; i < points.size() - 1; i++){
            std::vector<std::string> coords = explode(points[i], ",");
            // 坐标系转换，图片坐标转换到地图坐标
            float img_x = std::stof(coords[0]);
            float img_y = std::stof(coords[1]);
            float x_shift = img_x * m_map_resolution - abs(m_map_origin_x);
            float y_shift = (m_map_height - img_y) * m_map_resolution - abs(m_map_origin_y);
            float map_x = x_shift;
            float map_y = y_shift;
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = map_x;
            pose.pose.position.y = map_y;
            pose.pose.orientation.w = 1;
            task_msg.goal_list.push_back(pose);
        }
        std::vector<std::string> durations = explode(task_json["duration"], ";");
        for(int i = 0; i < durations.size() - 1; i++){
            task_msg.goal_duration_list.push_back(std::stoi(durations[i]));
        }

        if(task_json["cmd_pre"] != ""){
            std::cout << "Process cmd_pre field" << std::endl;
            std::vector<std::string> pres = explode(task_json["cmd_pre"], ";");
            for(int i = 0; i < pres.size() - 1; i++){
                task_msg.command_previous.push_back(pres[i]);
            }
        }

        if(task_json["cmd_post"] != ""){
            std::cout << "Process cmd_post field" << std::endl;
            std::vector<std::string> posts = explode(task_json["cmd_post"], ";");
            for(int i = 0; i < posts.size() - 1; i++){
                task_msg.command_back.push_back(posts[i]);
            }
        }

        task_msg.type = task_json["type"];  // 定时：0；即时：1
        task_msg.states = task_json["report_status"];
        task_msg.exec_time = task_json["exec_time"];
        task_msg.add_or_delete = 1;     // 0: delete; 1: add
        task_msg.resend_task = false;
        m_clean_task_pub.publish(task_msg);
    }
    // 处理机器人远程控制命令
    else if(msg->get_payload().substr(1, 4) == "\"op\""){
        // std::cout << "401" << std::endl;
        std::string raw_msg = msg->get_payload();
        json remote_control_json = json::parse(raw_msg);
        // std::cout << remote_control_json["op"] << std::endl;
        // std::cout << remote_control_json["topic"] << std::endl;
        // std::cout << remote_control_json["msg"] << std::endl;
        std::string data_msg = remote_control_json["msg"].dump();
        json data_json = json::parse(data_msg);
        // std::cout << data_json["data"] << std::endl;
        m_remote_control_cmd_pub = m_nh.advertise<std_msgs::String>(remote_control_json["topic"], 10);
        std_msgs::String msg;
        msg.data = data_json["data"];
        m_remote_control_cmd_pub.publish(msg);
    }
}

void WSServer::run()
{
    std::cout << "51 " << std::endl;
    m_server.set_access_channels(websocketpp::log::alevel::all);
    m_server.clear_access_channels(websocketpp::log::alevel::frame_payload);
    m_server.init_asio();

    m_server.set_message_handler(bind(&WSServer::on_message, this, &m_server, ::_1, ::_2));
    m_server.set_open_handler(bind(&WSServer::on_open, this, &m_server, ::_1));

    m_server.listen(m_ws_port);

    m_server.start_accept();
    m_server.run();
}