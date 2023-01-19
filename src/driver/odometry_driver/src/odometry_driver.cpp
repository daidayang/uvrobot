
#include "odometry_driver.hpp"
#include <fstream>
#include <iostream>
// #include <tf/transform_broadcaster.h>
// #include <nav_msgs/Odometry.h>


OdometryDriverRefactor::OdometryDriverRefactor() : private_nh("~")
{
    m_x = 0.0;
    m_y = 0.0;
    m_theta = 0;
    m_linear_v = 0.0;
    m_angular_v = 0.0;
    //两个轮间距
    m_wheel_track = 0.48;
    //轮子直径
    m_wheel_diameter = 0.27;
    //轮子每圈脉冲总数
    m_circle_enc = 40550;

    m_loop_time = 1000;
    m_current_pulse_right = 0;
    m_last_pulse_right = 0;

    m_current_pulse_left = 0;
    m_last_pulse_left = 0;

    m_first_get_encode_count = true;

    tf_time = ros::Time::now();

    private_nh.getParam("wheel_gauge", m_wheel_gauge);
    private_nh.getParam("port", m_port);
    private_nh.getParam("frame_id", m_frame_id);
    private_nh.getParam("baud_rate", m_baudrate);

    private_nh.getParam("wheel_track", m_wheel_track);
    private_nh.getParam("wheel_diameter", m_wheel_diameter);
    private_nh.getParam("circle_enc", m_circle_enc);

    std::cout<<"wheel_track   : " <<m_wheel_track<< std::endl;
    std::cout<<"wheel_diameter: " <<m_wheel_diameter<< std::endl;
    std::cout<<"circle_enc    : " <<m_circle_enc<< std::endl;

    init();
}

OdometryDriverRefactor::~OdometryDriverRefactor()
{   
    m_serial.close();
    ros::shutdown();
}
void OdometryDriverRefactor::init()
{   
//    m_odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);

//    m_laser_data_pub = nh.advertise<sensor_msgs::LaserScan> ("base_scan",100);

    m_twist_cmd_pub = nh.advertise<geometry_msgs::Twist>("twist_raw", 10);

    sub_pose = nh.subscribe("/initialpose", 10, &OdometryDriverRefactor::getClickPoseCallback,this);
    //获取左右轮速度,通过串口发送给arduino
    m_cmd_vel = nh.subscribe<geometry_msgs::Twist>("move_base/cmd_vel", 1, &OdometryDriverRefactor::cmdVelocityCallback, this);
    
    //m_pose_data = nh.subscribe<std_msgs::String>("ApDrvRx", 10, &OdometryDriverRefactor::poseDataCallback, this);

    //m_cmd_vel_pub = private_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
//    m_camera_odom_data = nh.subscribe<nav_msgs::Odometry>("camera/odom/sample", 10, &OdometryDriverRefactor::cameraOdomCallback, this);

//    m_laser_data = nh.subscribe<sensor_msgs::LaserScan>("scan", 10, &OdometryDriverRefactor::laserReceivedCallback, this);

    m_open_serial = false;

    // try 
    // { 
    //     //设置串口属性，并打开串口 
    //     m_serial.setPort(m_port); 
    //     m_serial.setBaudrate(m_baudrate); 
    //     serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
    //     m_serial.setTimeout(to); 
    //     m_serial.open();

    //     m_open_serial = true;
    // } 
    // catch (serial::IOException& e)
    // { 
    //     ROS_ERROR_STREAM("Unable to open port! ");
    //     m_open_serial = false;

    //     return;
    // } 

    // //检测串口是否已经打开，并给出提示信息 
    // if(m_serial.isOpen()) 
    // { 
    //     m_open_serial = true;
    //     ROS_INFO_STREAM("Serial Port initialized"); 
    // } 
    // else 
    // {   
    //     m_open_serial = false;
    //     return ; 
    // }

}

void OdometryDriverRefactor::getClickPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  std::cout<<"get click Pose Data: "<<msg->pose.pose.position.x<<","<<msg->pose.pose.position.y<<","<<msg->pose.pose.position.z<<std::endl;
}

void OdometryDriverRefactor::laserReceivedCallback(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
    sensor_msgs::LaserScan scan_msg = *laser_scan;

    scan_msg.header.stamp = ros::Time::now();
    //scan_msg.header.stamp.nsec = ((int)(scan_msg.header.stamp.nsec/1e8))*1e8;
    scan_msg.header.frame_id = "base_laser";

    m_laser_data_pub.publish(scan_msg);
    return;
}

void OdometryDriverRefactor::cameraOdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{


    //tf odom->
    m_trans.header.frame_id = "odom";
    m_trans.child_frame_id = "base_footprint";
    m_trans.transform.translation.x = msg->pose.pose.position.x;
    m_trans.transform.translation.y = msg->pose.pose.position.y;
    m_trans.transform.translation.z = msg->pose.pose.position.z;

    geometry_msgs::Quaternion odom_quat;
    odom_quat.x = msg->pose.pose.orientation.x;
    odom_quat.y = msg->pose.pose.orientation.y;
    odom_quat.z = msg->pose.pose.orientation.z;
    odom_quat.w = msg->pose.pose.orientation.w;
    m_trans.transform.rotation = odom_quat;
    
    m_trans.header.stamp = ros::Time::now();

    m_trans.header.stamp.nsec = ((int)(m_trans.header.stamp.nsec/1e8))*1e8;

    m_trans.header.seq = msg->header.seq;
    m_broadcaster.sendTransform(m_trans);

    tf_time = m_trans.header.stamp;
    //ROS_INFO("Send TF odom to base_footprint!");

    /////////////////////////////////////////////////////////////////////////
    m_trans.header.frame_id = "base_footprint";
    m_trans.child_frame_id = "base_link";
    m_trans.transform.translation.x = 0;
    m_trans.transform.translation.y = 0;
    m_trans.transform.translation.z = 0;

    //geometry_msgs::Quaternion odom_quat;
    odom_quat.x = 0;
    odom_quat.y = 0;
    odom_quat.z = 0;
    odom_quat.w = 1;
    m_trans.transform.rotation = odom_quat;
    
    m_trans.header.seq +=1;
    m_broadcaster.sendTransform(m_trans);


    //////////////////////////////////////////////////////////////////////////////
    m_trans.header.frame_id = "base_link";
    m_trans.child_frame_id = "base_laser";
    
    m_trans.transform.translation.x = 0.275;
    m_trans.transform.translation.y = 0;
    m_trans.transform.translation.z = 0.15;

    //geometry_msgs::Quaternion odom_quat;
    odom_quat.x = 0;
    odom_quat.y = 0;
    odom_quat.z = 0;
    odom_quat.w = 1;
    m_trans.transform.rotation = odom_quat;
    
    m_trans.header.seq += 1;
    m_broadcaster.sendTransform(m_trans);




    return;
}
void OdometryDriverRefactor::cmdVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{   

    if(m_wheel_gauge <= 0)
    {
        return;
    }
    //收到cmd_vel,把角速度转化为两轮的线速度
    double angular_v = msg->angular.z;
    double wheel_v_left =  msg->linear.x;// - angular_v*m_wheel_gauge/2;
    double wheel_v_right = msg->linear.x;// + angular_v*m_wheel_gauge/2;

    //std::string serial_string = "$"+std::to_string(wheel_v_left)+","+std::to_string(wheel_v_right);

    //std::cout<<"Calculate the wheel speed: "<< serial_string <<std::endl;
    
    // std_msgs::String velocity_wheel;
    // velocity_wheel.data = serial_string;
    // if(m_open_serial)
    // {
    //     m_serial.write(velocity_wheel.data);
    // }

    /////把速度转化为-128-128之间.[-0.5 - 0.5]
    double wheel_v_left_ch = wheel_v_left*128.0*2.0;
    
    double wheel_v_right_ch = wheel_v_right*128.0*2.0;

    geometry_msgs::Twist  twist_raw = *msg;

    twist_raw.linear.x = wheel_v_left_ch; //左轮速度
    twist_raw.linear.y = wheel_v_right_ch;//右轮速度
    
    m_twist_cmd_pub.publish(twist_raw);

    ROS_INFO("Calculation the speed: L %.3f, R %.3f (%.3f, %.3f), angular: %.3f",wheel_v_left_ch,wheel_v_right_ch,wheel_v_left,wheel_v_right,angular_v);

    return;
}

void OdometryDriverRefactor::poseDataCallback(const std_msgs::String::ConstPtr &msg)
{
    std::string str = msg->data;

    std::cout<<"get data: "+str.substr(1)<<std::endl;

    if(str.size()>0 && str[0] == '$')
    {
        //std::vector<std::string> data = split(str.substr(1),',');

        std::ofstream odometry_pose("odometry_pose.csv",std::ios::app);
        odometry_pose << str.substr(1);
        odometry_pose.close();

    }

    if(str.size()>0 && str[0] == 'P')
    {
        std::vector<std::string> data = split(str.substr(1),',');

        std::ofstream odometry_pause("odometry_pulse.csv",std::ios::app);
        odometry_pause << str.substr(1);
        odometry_pause.close();

        std::vector<double> odom_data;
        changePulseToOdomPose(data,odom_data);

        publishOdometry(odom_data);
    }

}

void OdometryDriverRefactor::changePulseToOdomPose(std::vector<std::string> &pulse_list,std::vector<double> &odometry_data)
{
    //read latest pulse number
    if(pulse_list.size()<2)
    {
        return;
    }
    
    double current_pulse_left  = std::stof(pulse_list[0]);
    double current_pulse_right = std::stof(pulse_list[1]);

    double cur_delta_r = 0;
    double cur_delta_l = 0;
    double delta_time  = 0;
    
     if(m_first_get_encode_count)
     {
       cur_delta_r = 0;
       cur_delta_l = 0;
       m_first_get_encode_count = false;
       
     }else
     {
        cur_delta_r = current_pulse_right - m_last_pulse_right;
        cur_delta_l = current_pulse_left - m_last_pulse_left;
     }
    
    delta_time = (double)m_loop_time/1000.0;
    
    m_last_pulse_right = current_pulse_right;
    m_last_pulse_left = current_pulse_left;
  
  
    double dright = 0;
    double dleft  = 0;
    
    if(cur_delta_r == 0 && cur_delta_l == 0)
    {
      dright = 0;
      dleft  = 0;
    
    }else
    {
      //每次采集周期行进的长度 =（当前脉冲数 - 上次脉冲数）*3.1415926 *轮胎直径/(轮子每圈脉冲总数,单位m)
      //右轮前进距离
      dright = cur_delta_r * M_PI * m_wheel_diameter/m_circle_enc;
      //左轮前进距离
      dleft = cur_delta_l * M_PI * m_wheel_diameter/m_circle_enc;
    }
  
  
    //机器人中心点的移动距离
    double dxy_ave = (dright + dleft)/2.0;
    //机器人中心点的转动角度
    double dth = (dright - dleft)/m_wheel_track;
    //速度
    double vxy = dxy_ave/delta_time;
    //角速度
    double vth = dth/delta_time;
    
  
    if(dxy_ave != 0)
    {
      double dx = cos(dth) * dxy_ave;
      double dy = -sin(dth) * dxy_ave;
      m_x += (cos(m_theta) * dx - sin(m_theta) * dy);
      m_y += (sin(m_theta) * dx + cos(m_theta) * dy);
    }
  
  
    if(dth != 0)
    {
      m_theta += dth;
      if(m_theta > M_PI)
      {
        m_theta = -1*M_PI;
      }
  
      if(m_theta < -1*M_PI)
      {
        m_theta = M_PI;
      }
    }

    odometry_data.clear();
    odometry_data.push_back(m_x);
    odometry_data.push_back(m_y);
    odometry_data.push_back(m_theta);
    odometry_data.push_back(vxy);
    odometry_data.push_back(vth);

    // std::ofstream odometry_pose_new("odometry_pose_new.csv",std::ios::app);
    // odometry_pose_new << m_x<<","<<m_y<<","<< m_theta <<"," << vxy <<","<< vth <<"\n";
    // odometry_pose_new.close();

    return;
}
void OdometryDriverRefactor::run()
{
    // if(accessCheck(m_port.c_str()) < 0)
    // {
    //     std::cout << "\033[1;31mFailed open port: " << std::strerror(errno) << "\033[0m" << std::endl;
    //     return;
    // }



    // char header_type[5];
    // unsigned char const const_header = '$';//0x24;
    // sensor_msgs::TimeReference gpstime_msg;

    // char *header = (char *)malloc(sizeof(char));
    // std::vector<char> vpayload;
    // int count = 0;
    // uint8_t buff[1];
    //while(true)
    //{
        //boost::asio::read(*m_sp, boost::asio::buffer(header, 1), boost::asio::transfer_at_least(1));
    //    if (m_serial.available()) 
    //     {
    //         m_serial.read(buff,1);//每次读取一个字符
        
    //         if(buff[0] == const_header)
    //         {
    //             if(count != 0)
    //             {
    //                 std::string report(vpayload.begin(), vpayload.end());
    //                 //剔除第一个字符后拆分
    //                 report.erase(report.begin());
    //                 std::vector<std::string> odom_data;
    //                 if(report.size()>1)
    //                 {
    //                     odom_data = split(report,',');//最后一位是回车符
    //                 }

    //                 if(odom_data.size() == 5)
    //                 {
                        
    //                     ROS_INFO("odom string: %s,%s,%s,%s,%s",odom_data[0].c_str(),odom_data[1].c_str(),odom_data[2].c_str(),odom_data[3].c_str(),odom_data[4].c_str());
    //                     publishOdometry(odom_data);


    //                     // geometry_msgs::Twist cmd_vel;
    //                     // cmd_vel.linear.x = 0.8;
    //                     // cmd_vel.linear.y = 0.0;
    //                     // cmd_vel.angular.z = 0.0;
    //                     // m_cmd_vel_pub.publish(cmd_vel);

    //                 }
    //             }

    //             count = 0;                  // 清空计数，为从头读取新的报文做准备
    //             vpayload.clear();           // 清空报文缓存
    //             //ros::Duration(0.1).sleep(); //是否需要??
    //         }


    //         vpayload.push_back(buff[0]);
    //         count++;
    //     }
    //     else
    //     {
    //         ros::Duration(0.2).sleep();
    //     }
        
    //    ros::spinOnce();
    //}
    ros::spin();
}

void OdometryDriverRefactor::publishOdometry(std::vector<std::string> &odom_data)
{
    int odom_data_count = 5;
    if(odom_data.size() != odom_data_count)
    {
        return;
    }

    std::vector<double> od_data;
    for(int i = 0;i<odom_data_count;++i)
    {
        double temp =std::stof(odom_data[i]);
        od_data.push_back(temp);
        //ROS_INFO("odom string to double: %f",temp);
    }    

    double x = od_data[0];
    double y = od_data[1];
    double theta = od_data[2];
    double linear_v = od_data[3];
    double angle_v =  od_data[4];
    //tf odom->base_link
    m_trans.header.frame_id = "odom";
    m_trans.child_frame_id = "base_link";
    m_trans.transform.translation.x = x;
    m_trans.transform.translation.y = y;

    geometry_msgs::Quaternion odom_quat;
    odom_quat.x = 0.0;
    odom_quat.y = 0.0;
    odom_quat.z = sin(theta / 2.0);
    odom_quat.w = cos(theta / 2.0);
    m_trans.transform.rotation = odom_quat;
    
    m_trans.header.stamp = ros::Time::now();
    m_broadcaster.sendTransform(m_trans);



  
  ////publish odom topic
  nav_msgs::Odometry odomtata;

  odomtata.header.frame_id = "odom";
  odomtata.child_frame_id = "base_link";
  
  odomtata.header.stamp = ros::Time::now();
  odomtata.pose.pose.position.x = x;
  odomtata.pose.pose.position.y = y;
  odomtata.pose.pose.position.z = 0;
  
  odomtata.pose.pose.orientation = odom_quat;
  
  ////liner velocity
  odomtata.twist.twist.linear.x = linear_v;
  odomtata.twist.twist.linear.y = 0;
  odomtata.twist.twist.linear.z = 0;
  
  ////angular velocity
  odomtata.twist.twist.angular.x = 0;
  odomtata.twist.twist.angular.y = 0;
  odomtata.twist.twist.angular.z = angle_v;
  
  m_odom_pub.publish(odomtata);

}


void OdometryDriverRefactor::publishOdometry(std::vector<double> &odom_data)
{
    int odom_data_count = 5;
    if(odom_data.size() != odom_data_count)
    {
        return;
    }

    std::vector<double> od_data = odom_data;
    // for(int i = 0;i<odom_data_count;++i)
    // {
    //     double temp =std::stof(odom_data[i]);
    //     od_data.push_back(temp);
    //     //ROS_INFO("odom string to double: %f",temp);
    // }
    
    double x = od_data[0];
    double y = od_data[1];
    double theta = od_data[2];
    double linear_v = od_data[3];
    double angle_v =  od_data[4];
    //tf odom->base_link
    m_trans.header.frame_id = "odom";
    m_trans.child_frame_id = "base_link";
    m_trans.transform.translation.x = x;
    m_trans.transform.translation.y = y;

    geometry_msgs::Quaternion odom_quat;
    odom_quat.x = 0.0;
    odom_quat.y = 0.0;
    odom_quat.z = sin(theta / 2.0);
    odom_quat.w = cos(theta / 2.0);
    m_trans.transform.rotation = odom_quat;
    
    m_trans.header.stamp = ros::Time::now();
    m_broadcaster.sendTransform(m_trans);



  
  ////publish odom topic
  nav_msgs::Odometry odomtata;

  odomtata.header.frame_id = "odom";
  odomtata.child_frame_id = "base_link";
  
  odomtata.header.stamp = ros::Time::now();
  odomtata.pose.pose.position.x = x;
  odomtata.pose.pose.position.y = y;
  odomtata.pose.pose.position.z = 0;
  
  odomtata.pose.pose.orientation = odom_quat;
  
  ////liner velocity
  odomtata.twist.twist.linear.x = linear_v;
  odomtata.twist.twist.linear.y = 0;
  odomtata.twist.twist.linear.z = 0;
  
  ////angular velocity
  odomtata.twist.twist.angular.x = 0;
  odomtata.twist.twist.angular.y = 0;
  odomtata.twist.twist.angular.z = angle_v;
  
  m_odom_pub.publish(odomtata);

}


std::vector<std::string> OdometryDriverRefactor::split(const std::string& str, char delim)
{
	std::stringstream ss(str);
	std::string s;
	std::vector<std::string> vec;
	while (std::getline(ss, s, delim))
		vec.push_back(s);

	if (!str.empty() && str.back() == delim)
		vec.push_back(std::string());

	return vec;
}
int OdometryDriverRefactor::accessCheck(std::string portname)
{
    int retval = access(portname.c_str(), R_OK);
    // if(retval < 0)
    //   std::cout << "\033[1;31mFailed open GPS: " << std::strerror(errno) << "\033[0m" << std::endl;

    return retval;
}

std::vector<std::string> OdometryDriverRefactor::explode(const std::string& text, const std::string& separators)
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
