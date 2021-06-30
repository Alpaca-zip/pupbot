#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "ros/time.h"
#include "trajectory_msgs/JointTrajectory.h"

class Vector2D{
  private:
  double double_x,double_y;

  public:
    Vector2D(double x,double y){
    this->double_x = x;
    this->double_y = y;
  }
  double x(){
    return double_x;
  }
  double y(){
    return double_y;
  }
};

class Vector{
  private:
  double double_x,double_y,double_z;

  public:
    Vector(double x,double y,double z){
    this->double_x = x;
    this->double_y = y;
    this->double_z = z;
  }
  double x(){
    return double_x;
  }
  double y(){
    return double_y;
  }
  double z(){
    return double_z;
  }
};

class Data{
  private:

  public:
  double x,y,z,a0,a1,b0;
  double angle1,angle2,angle3;
  double x_offset,y_offset,z_offset;
  double bone_length;
  double dir;
  bool c_inv;
  double target_leg_shoulder_joint, target_leg_upper_joint, target_leg_lower_joint;
  double l_inv[4][2] = {{1.0, -1.0},{-1.0, -1.0},{1.0, 1.0},{-1.0, 1.0}};
  trajectory_msgs::JointTrajectory leftfront_leg, leftback_leg, rightfront_leg, rightback_leg;
  void init() {
    leftfront_leg.joint_names.resize(3);
    leftfront_leg.points.resize(1);
    leftfront_leg.points[0].positions.resize(3);
    leftfront_leg.joint_names[0] ="leftfront_leg_shoulder_joint";
    leftfront_leg.joint_names[1] ="leftfront_leg_upper_joint";
    leftfront_leg.joint_names[2] ="leftfront_leg_lower_joint";
    leftback_leg.joint_names.resize(3);
    leftback_leg.points.resize(1);
    leftback_leg.points[0].positions.resize(3);
    leftback_leg.joint_names[0] ="leftback_leg_shoulder_joint";
    leftback_leg.joint_names[1] ="leftback_leg_upper_joint";
    leftback_leg.joint_names[2] ="leftback_leg_lower_joint";
    rightfront_leg.joint_names.resize(3);
    rightfront_leg.points.resize(1);
    rightfront_leg.points[0].positions.resize(3);
    rightfront_leg.joint_names[0] ="rightfront_leg_shoulder_joint";
    rightfront_leg.joint_names[1] ="rightfront_leg_upper_joint";
    rightfront_leg.joint_names[2] ="rightfront_leg_lower_joint";
    rightback_leg.joint_names.resize(3);
    rightback_leg.points.resize(1);
    rightback_leg.points[0].positions.resize(3);
    rightback_leg.joint_names[0] ="rightback_leg_shoulder_joint";
    rightback_leg.joint_names[1] ="rightback_leg_upper_joint";
    rightback_leg.joint_names[2] ="rightback_leg_lower_joint";
    x_offset=-14.0;
    y_offset=-6.0;
    z_offset=118.844;
    bone_length=78.0;
  }

  void trot(double c0_x,double c0_y,double dir_x,double dir_y,bool inv,double step_extent_x,double step_extent_y,double step_extent_z,double* vector_x,double* vector_y,double* vector_z) {
    double w0 = step_extent_x*0.1 / 2.0 * dir_x;
    double l0 = step_extent_y*0.1 * 4.0 * dir_y;
    double h0 = step_extent_z*0.1;
    if (inv==false) {
      c0_x=-c0_x;
      c0_y=-c0_y;
    }
    if (w0==0.0 && l0==0.0) {
      *vector_x = 0.0;
      *vector_y = 0.0;
      *vector_z = 0.0;
    } else if (w0==0.0) {
      double h1 = sqrt(abs((1.0 - (c0_y/l0)*(c0_y/l0)) * h0*h0));
      *vector_x = c0_x/0.1;
      *vector_y = c0_y/0.1;
      *vector_z = h1/0.1 * int(inv);
    } else if (l0==0.0) {
      double h1 = sqrt(abs((1.0 - (c0_x/w0)*(c0_x/w0)) * h0*h0));
      *vector_x = c0_x/0.1;
      *vector_y = c0_y/0.1;
      *vector_z = h1/0.1 * int(inv);
    } else {
      double h1 = sqrt(abs((1.0 - (c0_x/w0)*(c0_x/w0) - (c0_y/l0)*(c0_y/l0)) * h0*h0));
      *vector_x = c0_x/0.1;
      *vector_y = c0_y/0.1;
      *vector_z =h1/0.1 * int(inv);
    }
  }

  void count_c(int l, double dir_x, double dir_y, double step_extent_x, double c_iter[], double c[], double c_inv[]) {
    int number=22;
    double w0 = step_extent_x * 0.1 * std::max(abs(dir_x), abs(dir_y)) / 2.0;
    double a0 = (2.0 * w0) * (c_iter[l] / number) - w0;
    c[l] = a0;
    c_iter[l] += 1.0;
    if (c_iter[l] > number) {
      c[l] = -w0; 
      c_iter[l] = 1.0;

      c_inv[l] += 1.0;
      if (c_inv[l] > 31) c_inv[l] = 0.0;
    }
  }

  double rDir_x(double dir_x, double dir_y) {
    if (dir_x != 0.0 || dir_y != 0.0) {
      return dir_x/std::max(abs(dir_x), abs(dir_y));
    }else {
      return 1.0;
    } 
  }

  double rDir_y(double dir_x, double dir_y) {
    if (dir_x != 0.0 || dir_y != 0.0) {
      return dir_y/std::max(abs(dir_x), abs(dir_y));
    }else {
      return 1.0;
    } 
  }

  Data(){
    init();
  }
};