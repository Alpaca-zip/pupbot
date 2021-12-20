#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "trajectory_msgs/JointTrajectory.h"

#define X_OFFSET -6.0
#define BONE_LENGTH 83.0
#define TROT_STEP_EXTENT_X 30.0
#define TROT_STEP_EXTENT_Y 30.0
#define TROT_STEP_EXTENT_Z  8.0
#define LEFTFRONTLEG_Z_OFFSET 117
#define LEFTBACKLEG_Z_OFFSET 118
#define RIGHTFRONTLEG_Z_OFFSET 119
#define RIGHTBACKLEG_Z_OFFSET 119

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//crawl gait section
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//These options will be extended in a future release.
/*
#define CRAWL_STEP_EXTENT_X 40.0
#define CRAWL_STEP_EXTENT_Y 40.0
#define CRAWL_STEP_EXTENT_Z  7.0
*/
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

class Move_Pupbot{
  public:
  Move_Pupbot();
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//crawl gait section
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//These options will be extended in a future release.
/*
  int gait_state_num;
  void controlLoop_crawl();
*/
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  int gait_state_num;
  void controlLoop_trot();

  private:
  int l;
  int number;
  double x, y, z, a0, a1, b0;
  double angle1, angle2, angle3;
  double x_offset, z_offset_leftfront_leg, z_offset_leftback_leg, z_offset_rightfront_leg, z_offset_rightback_leg;
  double bone_length;
  double target_leg_shoulder_joint, target_left_leg_upper_joint, target_left_leg_lower_joint, target_right_leg_upper_joint, target_right_leg_lower_joint;
  double dirupdate_x;
  double turn0;
  double vector_x, vector_y, vector_z;
  double trot_step_extent_x, trot_step_extent_y, trot_step_extent_z;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//crawl gait section
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//These options will be extended in a future release.
/*
  int crawl_pattern[4] = {2, 0, 1, 3};
  int crawl_succession[4] = {2, 1, 3, 0};
  double crawl_step_extent_x, crawl_step_extent_y, crawl_step_extent_z;

  ros::Subscriber key_control_sub4;
  
  void crawl(double c0_x, double c0_y, bool inv, int i0);
  void gait_state_Callback(const std_msgs::Bool& gait_state);
*/
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
  ros::Subscriber sub_leftfront_leg_z_offset;
  ros::Subscriber sub_leftback_leg_z_offset;
  ros::Subscriber sub_rightfront_leg_z_offset;
  ros::Subscriber sub_rightback_leg_z_offset;
  trajectory_msgs::JointTrajectory leftfront_leg, leftback_leg, rightfront_leg, rightback_leg;

  void init();
  void trot(double c0_x, double c0_y, bool inv);
  void count_c(double step_extent_x);
  void key_controlCallback1(const std_msgs::Float64& direction_x);
  void key_controlCallback2(const std_msgs::Float64& turn);
  void startup_shutdown_Callback(const std_msgs::Bool& startup_shutdown);
  void leftfront_leg_z_offset_Callback(const std_msgs::Float64& leftfront_leg_z_offset);
  void leftback_leg_z_offset_Callback(const std_msgs::Float64& leftback_leg_z_offset);
  void rightfront_leg_z_offset_Callback(const std_msgs::Float64& rightfront_leg_z_offset);
  void rightback_leg_z_offset_Callback(const std_msgs::Float64& rightback_leg_z_offset);
  double rDir_x();
  double rDir_y();
};
