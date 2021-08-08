#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

class Dynamixel_Writer{
  public:
  Dynamixel_Writer();
  void controlLoop();

  private:
  const char *log;
  int baud_rate;
  int32_t goal_position[3];
  uint16_t model_number;
  uint8_t dxl_id[3] = {3, 21, 5};
  bool result;
  const char* port_name = "/dev/ttyACM0";
  const uint8_t handler_index = 0;
  DynamixelWorkbench dxl_wb;

  ros::NodeHandle nh;
  ros::Subscriber sub_joints;
  std_msgs::String joint_name[3];
  std_msgs::Float64 joint_pos[3];

  void init();
  void monitorJointState_callback(const sensor_msgs::JointState::ConstPtr& jointstate);
  void dxl_init();
  void dxl_torqueOn();
  void dxl_addSyncWriteHandler();
};