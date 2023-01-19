#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <asm/termbits.h> /* struct termios2 */
#include <unistd.h>
#include <sys/ioctl.h>
#include <time.h>
#include <stdarg.h>
#include <net/if.h>
#include <pthread.h>
#include <fstream>

#include "jazzy_drive.h"

JazzyDriver::JazzyDriver()
{
  std::string tmp_string;

  MsgCnt = 0;

  ros::NodeHandle private_nh("~");

  private_nh.param<std::string>("port", tty_path, "/dev/ttyACM0");
  ROS_INFO("tty_path = %s", tty_path.c_str());

//  private_nh.param<std::string>("motor_control_mode", tmp_string, "LeftRight");
  private_nh.param<std::string>("motor_control_mode", tmp_string, "AccelAngle");
//  tmp_string = std::toLower(tmp_string)
  ROS_INFO("motor_control_mode parm = %s", tmp_string.c_str());

  motor_control_mode = 0;
  if (tmp_string.compare("LeftRight") == 0)
    motor_control_mode = 1;
  ROS_INFO("motor_control_mode = %i", motor_control_mode);

  m_UVRobot_state_pub_ = nh_.advertise<std_msgs::Int32>("UVRobot_state", 10);
  m_UVRobot_state_sub_ = nh_.subscribe<std_msgs::Int32>("UVRobot_state", 10, &JazzyDriver::getCurrentStateCallback, this);

  apdriverx_pub_ = nh_.advertise<std_msgs::String>("ApDrvRx", 1);
  apdrivetx_pub_ = nh_.advertise<std_msgs::String>("ApDrvTx", 1);
  twist_sub_ = nh_.subscribe<geometry_msgs::Twist>("twist_raw", 10, &JazzyDriver::TwistCallback, this);
  xbox_sub_ = nh_.subscribe<std_msgs::String>("xbox", 10, &JazzyDriver::RemoteDrivingCallback, this);

  RobotDriveMode = WORKING;
}

void JazzyDriver::getCurrentStateCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int sta = msg->data;
    switch (sta)
    {
        case 1:
            RobotDriveMode = WORKING;
            break;
        case 2:
            RobotDriveMode = CHARGING;
            break;
        case 3:
            RobotDriveMode = IDLE;
            break;
        case 4:
            RobotDriveMode = FAULT;
            break;
        case 5:
            RobotDriveMode = OFFCHARGING;
            break;

        default:
            break;
    }
    std::cout << "get uvrobot mode...... " << sta << std::endl;
}

void JazzyDriver::publishState(UVRobotWorkState sta)
{   
    std_msgs::Int32 stas;
    int state_num = -1;
    switch (sta)
    {
        case WORKING:
            state_num = 1;
            break;
        case CHARGING:
            state_num = 2;
            break;
        case IDLE:
            state_num = 3;
            break;
        case FAULT:
            state_num = 4;
            break;
        case OFFCHARGING:
            state_num = 5;
            break;
        default:
            break;
    }
   
   stas.data = state_num;
   m_UVRobot_state_pub_.publish(stas);
}

void JazzyDriver::RelayJazzyMsg(std::string msg)
{
  std_msgs::String cmd;
  cmd.data = msg;
  apdriverx_pub_.publish(cmd);

  switch( RobotDriveMode )
  {
    case CHARGING:
      if ( msg == "SR0" ) {
        count_SetChargingModeMsgSent=0;
        publishState(WORKING);
      }
    break;

    case WORKING:
      if ( msg == "SR1" ) {
        count_SetChargingModeMsgSent++;
        if ( count_SetChargingModeMsgSent<3 )
        {
          publishState(CHARGING);
        }
      }
    break;
  }
}


void JazzyDriver::RemoteDrivingCallback(const std_msgs::String::ConstPtr& msg)
{
  int result, frame_len;
  char Accel;
  char Brake;
  char bH;
  char bL;

  std::string val;
  val = msg->data + "\n";

  frame_len = val.size();
  char cstr[frame_len];
  strcpy(cstr, val.c_str());	// or pass &s[0]
//  std::cout << "cmd_vel: " << cstr;

  cstr[1]=IntToChar((MsgCnt++)%16);
  result = (int) write(tty_fd, cstr, (size_t) frame_len);

  if (result == -1) {
        ROS_ERROR("write() failed: %s", strerror(errno));
  }

//  ROS_INFO("cmd_vel: %s", val);
//  SendDrivingCmd(0, Angle, Accel, Brake);
}

void JazzyDriver::TwistCallback(const geometry_msgs::TwistConstPtr &msg)
{
  char buffer [7];
  std::string cmd_str = " twist_cmd " + std::to_string(msg->angular.x) + " " + std::to_string(msg->angular.y);
  std::string pose_str = "";
//  int len = 0;
  int result;
  size_t str_len;

//  pose_str =  " pose "+ std::to_string(current_pose.position.x) +" "+ std::to_string(current_pose.position.y) + " " + std::to_string(current_pose.position.z);
//  cmd_str += pose_str;

  if (RobotDriveMode == WORKING || RobotDriveMode == CHARGING ) {
    if ( motor_control_mode == 1 ) {
    }
    else {
       int Accel = 0;
       if ( abs( msg->linear.x ) >= 1 ) {
         if ( msg->linear.x > 0 ) {
           Accel = msg->linear.x / 2;
           if( Accel > 100 )
             Accel = 100;
         }
         else {
           Accel = msg->linear.x / 2;
           if( Accel < -100 )
             Accel = -100;
         }
       }
       Accel = Accel + 128;

       int Angel = 0;
       if ( abs( msg->angular.z ) >= 0.01 ) {  // 5 degree
           Angel = msg->angular.z * 50.0 / 0.4;
           if (Angel > 100) Angel = 100;
           if (Angel < -100) Angel = -100;
	   Angel = -Angel;
       }
       Angel = Angel + 128;

       ROS_INFO("Auto  F: %i, T: %i", Accel, Angel);

       sprintf (buffer, "D1%02X%02X", Accel, Angel );
       buffer[6] = '\n';

       pose_str = buffer;
       str_len = pose_str.length();
       ROS_INFO("Drv : %s, %d", pose_str.c_str(), (int)str_len);

       buffer[1]=IntToChar((MsgCnt++)%16);
       result = (int) write(tty_fd, buffer, str_len);

       if (result == -1) {
         ROS_ERROR("write() failed: %s", strerror(errno));
       }
    }
  }
  else {
    ROS_INFO("RobotDriveMode = STOP.  Twist_cmd ignored");
  }
}


char JazzyDriver::IntToChar(int a)
{
  if ( a >=0 && a <= 9) 
    return '0' + a;
  if ( a >= 10 && a <= 15 )
    return 'A' + a - 10;
  return '0';
}


int JazzyDriver::init_serial_adapter(char *tty_device, int baudrate) {
    int result;
    struct termios2 tio;

    ROS_INFO("init_serial_adapter(%s, %d)", tty_device, baudrate);

    tty_fd = open(tty_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (tty_fd == -1) {
        ROS_ERROR("open(%s) failed: %s\n", tty_device, strerror(errno));
        return -1;
    }
    ROS_INFO("open(%s) OK", tty_device);

    result = ioctl(tty_fd, TCGETS2, &tio);
    if (result == -1) {
        ROS_ERROR("ioctl() failed: %s\n", strerror(errno));
        close(tty_fd);
        return -1;
    }
    ROS_INFO("ioctl() OK");

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag = BOTHER | CS8 | CSTOPB;
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_ispeed = (speed_t) baudrate;
    tio.c_ospeed = (speed_t) baudrate;

    result = ioctl(tty_fd, TCSETS2, &tio);
    if (result == -1) {
        ROS_ERROR("ioctl() failed: %s\n", strerror(errno));
        close(tty_fd);
        return -1;
    }
    ROS_INFO("ioctl() 2 OK");

    return tty_fd;
}

int JazzyDriver::Init(std::string name) {
    int baud_rate;

    InstanceName = name;
    baud_rate = 57600;

    ROS_INFO("tty_path.2 = %s", tty_path.c_str());

    // tty_path = "/dev/canbus";
    char *tty_path_ptr = (char *)tty_path.c_str();
    tty_fd = init_serial_adapter(tty_path_ptr, baud_rate);
    ROS_INFO("tty_fd %d", tty_fd);
    if (tty_fd == -1) {
        return EXIT_FAILURE;
    }
    return 0;
}

void JazzyDriver::teardown() {
    if (0 != tty_fd) {
        close(tty_fd);
        ROS_INFO("close(tty_fd)");
    }
}

void *tty_read_thread(void *obj)
{
  std_msgs::String cmd;
  int RetryInit = 3;
  int RcStatus = 0;
  JazzyDriver *candrv_;
  candrv_ = (JazzyDriver*) obj;

  ROS_INFO("Enter CAN read thread [%s]", candrv_->InstanceName.c_str());

  ros::Rate loop_rate(500);

  while(ros::ok())
  {
    std::string response;
    char byte_read[1];
    char buffer_read[256];
    int frame_len2 = 0;
    char* buffer_xfm;

    do
    {
      int n = read(candrv_->tty_fd, byte_read, 1);
      if (n > 0) {
        buffer_read[frame_len2] = byte_read[0];
        frame_len2 = frame_len2 + n;
//        response += std::string(buffer);
//        std::cout << std::hex << (int) byte_read[0];
//        std::cout << response.length() << "-" << std::hex << buffer[0] << " ";
//        ROS_INFO("RX[%]", buffer[0]);
      }
    } while (byte_read[0] != '\n' && frame_len2 < 256); // 'X' means end of transmission

    buffer_xfm = new char[frame_len2+1];
    for(int idx=0; idx<frame_len2; idx++)
    {
//       std::cout << " " << std::hex << (int) buffer_read[idx];
       buffer_xfm[idx] = buffer_read[idx];
    }
    buffer_xfm[frame_len2] = '\0';
    response = std::string(buffer_xfm);

    if ( frame_len2 > 1 ) {
//      std::cout << "Len: " << response.length() << " " << response;
//      std::cout << std::endl << "Len1: " << response.length() << " Len2: " << frame_len2 << "  "  << response << std::endl;
      candrv_->RelayJazzyMsg(response);
    }

    frame_len2 = 0;
//    response = null;
    delete [] buffer_xfm;
//    buffer_xfm = null;

    if ( RcStatus < 0) {
        RetryInit--;
        if ( RetryInit >= 0 ) {
            candrv_->teardown();
            candrv_->Init("drive1");
        }
    }

    loop_rate.sleep();
    ros::spinOnce();
  }

  ROS_INFO("Exit CAN read thread");

  pthread_exit(NULL);
}

int main(int argc, char** argv)
{
  clock_t t;

  ros::init(argc, argv, "jazzy_drive");

  ros::Time::init();
  t = clock() + CLOCKS_PER_SEC;

  JazzyDriver candrv;
  candrv.Init("drive1");

  pthread_t thread1;
  pthread_create(&thread1,NULL,tty_read_thread,&candrv);

  ros::Rate r(50); // 50 hz

  while (ros::ok())
  {
    if ( clock() > t ) {
      t += CLOCKS_PER_SEC;
    }

    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("Start teardown()");
}
