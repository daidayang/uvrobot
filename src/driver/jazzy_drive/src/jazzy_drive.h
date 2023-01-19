#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>

#ifndef JAZZY_DRIVE_H
#define JAZZY_DRIVE_H


enum UVRobotWorkState {
    WORKING     = 1,
    CHARGING    = 2,
    IDLE        = 3,
    FAULT       = 4,
    OFFCHARGING = 5
  };

#define RECV_STACK_SIZE 128

typedef enum {
    RECEIVING,
    COMPLETE,
    MISSED_HEADER
} FRAME_STATE;

class JazzyDriver
{
public:
  JazzyDriver();

  int Init(std::string name);
  void teardown();
  std::string InstanceName = "None";
  int tty_fd = 0;
  void RelayJazzyMsg(std::string msg);

private:
  void publishState(UVRobotWorkState sta);
  void RemoteDrivingCallback(const std_msgs::String::ConstPtr& msg);
  void TwistCallback(const geometry_msgs::TwistConstPtr &msg);
  int init_serial_adapter(char *tty_device, int baudrate);
  void getCurrentStateCallback(const std_msgs::Int32::ConstPtr& msg);
  char IntToChar(int a);
  ros::NodeHandle nh_;

// Serial
  std::string tty_path;
  int MsgCnt;

  geometry_msgs::Pose current_pose;

  fd_set readSet;
  int recvStackElements = 0;
  char recvStack[RECV_STACK_SIZE][8];
  pthread_mutex_t recvStackMutex = PTHREAD_MUTEX_INITIALIZER;

  UVRobotWorkState RobotDriveMode;

  int count_SetChargingModeMsgSent;
  std::string prefix;       // Added by Lor, for EPS_Zero_Pos saving.

  float OldX;
  float OldY;
  int OldBtn1;
  int motor_control_mode;  // 0: Accel/Angle,  1: Left/Right

  int OldAngle;
  int OldAccel;
  int OldBrake;

  struct timeval timeout = {1, 0};

  ros::Publisher apdriverx_pub_;
  ros::Publisher apdrivetx_pub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber xbox_sub_;
  ros::Publisher  m_UVRobot_state_pub_;
  ros::Subscriber m_UVRobot_state_sub_;
};

#endif	//	JAZZY_DRIVE_H
