#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

#define PORT_NAME "/dev/ttyACM0"
#define MODEL_NUMBER 12
#define BAUD_RATE 1000000

#define LEFTFRONT_LEG_SHOULDER_ID 3
#define LEFTFRONT_LEG_UPPER_ID 21
#define LEFTFRONT_LEG_LOWER_ID 5

class Dynamixel_Writer{
  public:
  Dynamixel_Writer();
  void controlLoop();

  private:
  const char *log;
  int baud_rate;
  int32_t goal_position[3];
  uint16_t model_number;
  uint8_t dxl_id[3] = {LEFTFRONT_LEG_SHOULDER_ID, LEFTFRONT_LEG_UPPER_ID, LEFTFRONT_LEG_LOWER_ID};
  bool result;
  const char* port_name = PORT_NAME;
  const uint8_t handler_index = 0;
  DynamixelWorkbench dxl_wb;

  ros::NodeHandle nh;
  ros::Subscriber sub_leftfront_leg;
  std_msgs::String joint_name[3];
  std_msgs::Float64 joint_pos[3];

  void init();
  void monitor_leftfront_leg_callback(const trajectory_msgs::JointTrajectory& leftfront_leg);
  void dxl_init();
  void dxl_torqueOn();
  void dxl_addSyncWriteHandler();
};