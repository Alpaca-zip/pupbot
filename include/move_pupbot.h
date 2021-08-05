#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "ros/time.h"
#include "trajectory_msgs/JointTrajectory.h"

class Vector2D{
  private:
  double double_x,double_y;

  public:
  Vector2D(double x,double y);
  double x();
  double y();
};

class Vector{
  private:
  double double_x,double_y,double_z;

  public:
  Vector(double x,double y,double z);
  double x();
  double y();
  double z();
};

class Data{
  private:

  public:
  double x,y,z,a0,a1,b0;
  double angle1,angle2,angle3;
  double x_offset,y_offset,z_offset;
  double bone_length;
  double dir;
  double target_leg_shoulder_joint, target_left_leg_upper_joint, target_left_leg_lower_joint, target_right_leg_upper_joint, target_right_leg_lower_joint;
  double l_inv[4][2] = {{1.0, -1.0},{-1.0, -1.0},{1.0, 1.0},{-1.0, 1.0}};
  double dirupdate_x;
  double turn0;
  bool c_inv;
  bool startup_shutdown_bool;
  
  ros::NodeHandle nh;
  ros::Publisher pub_leftfront_leg;
  ros::Publisher pub_leftback_leg;
  ros::Publisher pub_rightfront_leg;
  ros::Publisher pub_rightback_leg;
  ros::Subscriber key_control_sub1;
  ros::Subscriber key_control_sub2;
  ros::Subscriber key_control_sub3;
  trajectory_msgs::JointTrajectory leftfront_leg, leftback_leg, rightfront_leg, rightback_leg;

  Data();
  void init();
  void trot(double c0_x,double c0_y,double dir_x,double dir_y,bool inv,double step_extent_x,double step_extent_y,double step_extent_z,double* vector_x,double* vector_y,double* vector_z);
  void count_c(int l, double dir_x, double dir_y, double step_extent_x, double c_iter[], double c[], double c_inv[]);
  void key_controlCallback1(const std_msgs::Float64& direction_x);
  void key_controlCallback2(const std_msgs::Float64& turn);
  void startup_shutdown_Callback(const std_msgs::Bool& startup_shutdown);
  double rDir_x(double dir_x,double dir_y);
  double rDir_y(double dir_x,double dir_y);
};