#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

#define PORT_NAME "/dev/ttyACM1"
#define MODEL_NUMBER 12
#define BAUD_RATE 1000000

#define LEFTFRONT_LEG_SHOULDER_ID 3
#define LEFTFRONT_LEG_UPPER_ID 21
#define LEFTFRONT_LEG_LOWER_ID 5
#define RIGHTFRONT_LEG_SHOULDER_ID 2
#define RIGHTFRONT_LEG_UPPER_ID 1
#define RIGHTFRONT_LEG_LOWER_ID 13
#define LEFTBACK_LEG_SHOULDER_ID 16
#define LEFTBACK_LEG_UPPER_ID 8
#define LEFTBACK_LEG_LOWER_ID 6
#define RIGHTBACK_LEG_SHOULDER_ID 7
#define RIGHTBACK_LEG_UPPER_ID 18
#define RIGHTBACK_LEG_LOWER_ID 14

class Dynamixel_Controller{
  public:
  Dynamixel_Controller();
  void controlLoop();

  private:
  const char *log;
  const char* port_name = PORT_NAME;
  int baud_rate;
  int32_t goal_position[12];
  int32_t getdata_from_dynamixel;
  uint16_t model_number;
  uint8_t dxl_id[12] = {LEFTFRONT_LEG_SHOULDER_ID, LEFTFRONT_LEG_UPPER_ID, LEFTFRONT_LEG_LOWER_ID, RIGHTFRONT_LEG_SHOULDER_ID, RIGHTFRONT_LEG_UPPER_ID, RIGHTFRONT_LEG_LOWER_ID, LEFTBACK_LEG_SHOULDER_ID, LEFTBACK_LEG_UPPER_ID, LEFTBACK_LEG_LOWER_ID, RIGHTBACK_LEG_SHOULDER_ID, RIGHTBACK_LEG_UPPER_ID, RIGHTBACK_LEG_LOWER_ID};
  bool result;
  const uint8_t handler_index = 0;
  DynamixelWorkbench dxl_wb;

  ros::NodeHandle nh;
  ros::Publisher dynamixel_state_leftfront_leg;
  ros::Publisher dynamixel_state_leftback_leg;
  ros::Publisher dynamixel_state_rightfront_leg;
  ros::Publisher dynamixel_state_rightback_leg;
  ros::Subscriber sub_leftfront_leg;
  ros::Subscriber sub_rightfront_leg;
  ros::Subscriber sub_leftback_leg;
  ros::Subscriber sub_rightback_leg;
  ros::Subscriber sub_gait_stop;
  std_msgs::String joint_name[12];
  std_msgs::Float64 joint_pos[12];
  std_msgs::Int32 leftfront_leg_load, leftback_leg_load, rightfront_leg_load, rightback_leg_load;

  void init();
  void monitor_leftfront_leg_callback(const trajectory_msgs::JointTrajectory& leftfront_leg);
  void monitor_rightfront_leg_callback(const trajectory_msgs::JointTrajectory& rightfront_leg);
  void monitor_leftback_leg_callback(const trajectory_msgs::JointTrajectory& leftback_leg);
  void monitor_rightback_leg_callback(const trajectory_msgs::JointTrajectory& leftback_leg);
  void monitor_load_callback(const std_msgs::Bool& gait_stop_bool);
  void dxl_init();
  void dxl_torqueOn();
  void dxl_addSyncWriteHandler();
};