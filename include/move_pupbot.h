#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "ros/time.h"
#include "trajectory_msgs/JointTrajectory.h"

#define X_OFFSET 0.0
#define Z_OFFSET 120
#define BONE_LENGTH 83.0
#define STEP_EXTENT_X 40
#define STEP_EXTENT_Y 40
#define STEP_EXTENT_Z  7

class Move_Pupbot{
  public:
  Move_Pupbot();
  void controlLoop();

  private:
  int l;
  int number;
  int gait_state_num;
  int crawl_pattern[4] = {2, 0, 1, 3};
  int crawl_succession[4] = {2, 1, 3, 0};
  double x, y, z, a0, a1, b0;
  double angle1, angle2, angle3;
  double x_offset, z_offset;
  double bone_length;
  double target_leg_shoulder_joint, target_left_leg_upper_joint, target_left_leg_lower_joint, target_right_leg_upper_joint, target_right_leg_lower_joint;
  double dirupdate_x;
  double turn0;
  double vector_x, vector_y, vector_z;
  double step_extent_x, step_extent_y, step_extent_z;
  double dir_x, dir_y;
  double w0, l0, h0;
  double w0_count_c, a0_count_c;
  double startup_shutdown_upper_left, startup_shutdown_lower_left, startup_shutdown_upper_right, startup_shutdown_lower_right;
  double l_inv[4][2] = {{1.0, -1.0}, {-1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}};
  double c_iter[4] = {0.0, 0.0, 0.0, 0.0};
  double c[4] = {0.0, 0.0, 0.0, 0.0};
  double c_inv[4] = {0.0, 0.0, 0.0, 0.0};
  bool startup_shutdown_bool;
  
  ros::NodeHandle nh;
  ros::Publisher pub_leftfront_leg;
  ros::Publisher pub_leftback_leg;
  ros::Publisher pub_rightfront_leg;
  ros::Publisher pub_rightback_leg;
  ros::Subscriber key_control_sub1;
  ros::Subscriber key_control_sub2;
  ros::Subscriber key_control_sub3;
  ros::Subscriber key_control_sub4;
  trajectory_msgs::JointTrajectory leftfront_leg, leftback_leg, rightfront_leg, rightback_leg;

  void init();
  void trot(double c0_x, double c0_y, bool inv);
  void crawl(double c0_x, double c0_y, bool inv, int i0);
  void count_c();
  void key_controlCallback1(const std_msgs::Float64& direction_x);
  void key_controlCallback2(const std_msgs::Float64& turn);
  void startup_shutdown_Callback(const std_msgs::Bool& startup_shutdown);
  void gait_state_Callback(const std_msgs::Bool& gait_state);
  double rDir_x();
  double rDir_y();
};
