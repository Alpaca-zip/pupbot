#include "move_quadruped_robot.h"

ros::Publisher pub_leftfront_leg;
ros::Publisher pub_leftback_leg;
ros::Publisher pub_rightfront_leg;
ros::Publisher pub_rightback_leg;
ros::Subscriber key_control_sub1;
ros::Subscriber key_control_sub2;
ros::Subscriber key_control_sub3;
double dirupdate_x=0.0;
double turn0=0.0;
bool startup_shutdown_bool=false;

void key_controlCallback1(const std_msgs::Float64& direction_x) {
  dirupdate_x=direction_x.data;
}

void key_controlCallback2(const std_msgs::Float64& turn) {
  turn0=turn.data;
}

void startup_shutdown_Callback(const std_msgs::Bool& startup_shutdown) {
  Data ssc;
  double startup_shutdown_upper = 0.0;
  double startup_shutdown_lower = 0.0;
  if (startup_shutdown_bool==false) {
    startup_shutdown_upper = 0.7;
    startup_shutdown_lower = -1.5;
    for (int i=0;i<22;i++) {
      startup_shutdown_upper -= 0.7/22.0;
      startup_shutdown_lower += 1.5/22.0;
      ssc.leftfront_leg.points[0].positions[0] = 0.0;
      ssc.leftfront_leg.points[0].positions[1] = startup_shutdown_upper;
      ssc.leftfront_leg.points[0].positions[2] = startup_shutdown_lower;
      ssc.leftfront_leg.header.stamp = ros::Time::now();
      ssc.leftfront_leg.points[0].time_from_start = ros::Duration(0.1);
      pub_leftfront_leg.publish(ssc.leftfront_leg);    
      ssc.leftback_leg.points[0].positions[0] = 0.0;
      ssc.leftback_leg.points[0].positions[1] = startup_shutdown_upper;
      ssc.leftback_leg.points[0].positions[2] = startup_shutdown_lower;
      ssc.leftback_leg.header.stamp = ros::Time::now();
      ssc.leftback_leg.points[0].time_from_start = ros::Duration(0.1);
      pub_leftback_leg.publish(ssc.leftback_leg);
      ssc.rightback_leg.points[0].positions[0] = 0.0;
      ssc.rightback_leg.points[0].positions[1] = startup_shutdown_upper;
      ssc.rightback_leg.points[0].positions[2] = startup_shutdown_lower;
      ssc.rightback_leg.header.stamp = ros::Time::now();
      ssc.rightback_leg.points[0].time_from_start = ros::Duration(0.1);
      pub_rightback_leg.publish(ssc.rightback_leg);
      ssc.rightfront_leg.points[0].positions[0] = 0.0;
      ssc.rightfront_leg.points[0].positions[1] = startup_shutdown_upper;
      ssc.rightfront_leg.points[0].positions[2] = startup_shutdown_lower;
      ssc.rightfront_leg.header.stamp = ros::Time::now();
      ssc.rightfront_leg.points[0].time_from_start = ros::Duration(0.1);
      pub_rightfront_leg.publish(ssc.rightfront_leg); 
      ros::Duration(0.1).sleep();
    }
    startup_shutdown_bool=true;
  }else if (startup_shutdown_bool==true) {
    startup_shutdown_upper = 0.0;
    startup_shutdown_lower = 0.0;
    for (int i=0;i<22;i++) {
      startup_shutdown_upper += 0.7/22.0;
      startup_shutdown_lower -= 1.5/22.0;
      ssc.leftfront_leg.points[0].positions[0] = 0.0;
      ssc.leftfront_leg.points[0].positions[1] = startup_shutdown_upper;
      ssc.leftfront_leg.points[0].positions[2] = startup_shutdown_lower;
      ssc.leftfront_leg.header.stamp = ros::Time::now();
      ssc.leftfront_leg.points[0].time_from_start = ros::Duration(0.1);
      pub_leftfront_leg.publish(ssc.leftfront_leg);    
      ssc.leftback_leg.points[0].positions[0] = 0.0;
      ssc.leftback_leg.points[0].positions[1] = startup_shutdown_upper;
      ssc.leftback_leg.points[0].positions[2] = startup_shutdown_lower;
      ssc.leftback_leg.header.stamp = ros::Time::now();
      ssc.leftback_leg.points[0].time_from_start = ros::Duration(0.1);
      pub_leftback_leg.publish(ssc.leftback_leg);
      ssc.rightback_leg.points[0].positions[0] = 0.0;
      ssc.rightback_leg.points[0].positions[1] = startup_shutdown_upper;
      ssc.rightback_leg.points[0].positions[2] = startup_shutdown_lower;
      ssc.rightback_leg.header.stamp = ros::Time::now();
      ssc.rightback_leg.points[0].time_from_start = ros::Duration(0.1);
      pub_rightback_leg.publish(ssc.rightback_leg);
      ssc.rightfront_leg.points[0].positions[0] = 0.0;
      ssc.rightfront_leg.points[0].positions[1] = startup_shutdown_upper;
      ssc.rightfront_leg.points[0].positions[2] = startup_shutdown_lower;
      ssc.rightfront_leg.header.stamp = ros::Time::now();
      ssc.rightfront_leg.points[0].time_from_start = ros::Duration(0.1);
      pub_rightfront_leg.publish(ssc.rightfront_leg); 
      ros::Duration(0.1).sleep();
    }
    startup_shutdown_bool=false;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_quadruped_robot");
  ros::NodeHandle nh;
  ros::Rate loop_rate(50);
  pub_leftfront_leg = nh.advertise<trajectory_msgs::JointTrajectory>("/leftfront_leg_controller/command", 10);
  pub_leftback_leg = nh.advertise<trajectory_msgs::JointTrajectory>("/leftback_leg_controller/command", 10);
  pub_rightfront_leg = nh.advertise<trajectory_msgs::JointTrajectory>("/rightfront_leg_controller/command", 10);
  pub_rightback_leg = nh.advertise<trajectory_msgs::JointTrajectory>("/rightback_leg_controller/command", 10);
  key_control_sub1 = nh.subscribe("key_control1", 10, key_controlCallback1);
  key_control_sub2 = nh.subscribe("key_control2", 10, startup_shutdown_Callback);
  key_control_sub3 = nh.subscribe("key_control3", 10, key_controlCallback2);

  Vector step_extent(40, 40, 15);
  Vector2D direction(0, 0);
  Vector vector(0, 0, 0);
  Data data;
  int l=0;
  double c_iter[4];
  double c[4];
  double c_inv[4];
  double vector_x,vector_y,vector_z;

  while (ros::ok()) {

    for (l=0;l<4;l++) {
      if (startup_shutdown_bool==false) continue;
      Vector2D direction(dirupdate_x+turn0*data.l_inv[l][1], turn0*data.l_inv[l][0]);
      data.count_c(l, direction.x(), direction.y(), step_extent.x(), c_iter, c, c_inv);
      data.trot(data.rDir_x(direction.x(),direction.y())*c[l], data.l_inv[l][1]*data.rDir_y(direction.x(),direction.y())*c[l], direction.x(), direction.y(), bool(l%2) ^ bool(fmod(c_inv[l],2.0)), step_extent.x(), step_extent.y(), step_extent.z(), &vector_x, &vector_y, &vector_z);
      Vector vector(vector_x, vector_y, vector_z);
      data.x = data.x_offset+vector.x();
      data.y = vector.y();
      data.z = data.z_offset-vector.z();
      
      data.a0 = (atan(data.x/data.z))*180.0/M_PI;
      data.a1 = (atan(data.y/data.z))*180.0/M_PI;
      data.b0 = sqrt(data.x*data.x+data.z*data.z);

      data.angle1 = data.a1;
      data.angle3 = (asin((data.b0/2.0)/data.bone_length))*180.0/M_PI*2.0;
      data.angle2 = data.angle3/2.0+data.a0;

      data.target_leg_shoulder_joint = data.angle1*M_PI/180.0;
      data.target_leg_upper_joint = (45.0-data.angle2)*M_PI/180.0;
      data.target_leg_lower_joint = (data.angle3-90.0)*M_PI/180.0;
      ROS_INFO("angle:shoulder=%lf upper=%lf lower=%lf",data.angle1, data.angle2, data.angle3);
      if (l==0) {
        data.leftfront_leg.points[0].positions[0] = data.target_leg_shoulder_joint;
        data.leftfront_leg.points[0].positions[1] = data.target_leg_upper_joint;
        data.leftfront_leg.points[0].positions[2] = data.target_leg_lower_joint;
        data.leftfront_leg.header.stamp = ros::Time::now();
        data.leftfront_leg.points[0].time_from_start = ros::Duration(0.02);
        pub_leftfront_leg.publish(data.leftfront_leg);    
      } else if (l==1) {
        data.leftback_leg.points[0].positions[0] = data.target_leg_shoulder_joint;
        data.leftback_leg.points[0].positions[1] = data.target_leg_upper_joint;
        data.leftback_leg.points[0].positions[2] = data.target_leg_lower_joint;
        data.leftback_leg.header.stamp = ros::Time::now();
        data.leftback_leg.points[0].time_from_start = ros::Duration(0.02);
        pub_leftback_leg.publish(data.leftback_leg);    
      } else if (l==2) {
        data.rightback_leg.points[0].positions[0] = data.target_leg_shoulder_joint;
        data.rightback_leg.points[0].positions[1] = data.target_leg_upper_joint;
        data.rightback_leg.points[0].positions[2] = data.target_leg_lower_joint;
        data.rightback_leg.header.stamp = ros::Time::now();
        data.rightback_leg.points[0].time_from_start = ros::Duration(0.02);
        pub_rightback_leg.publish(data.rightback_leg);    
      } else if (l==3) {
        data.rightfront_leg.points[0].positions[0] = data.target_leg_shoulder_joint;
        data.rightfront_leg.points[0].positions[1] = data.target_leg_upper_joint;
        data.rightfront_leg.points[0].positions[2] = data.target_leg_lower_joint;
        data.rightfront_leg.header.stamp = ros::Time::now();
        data.rightfront_leg.points[0].time_from_start = ros::Duration(0.02);
        pub_rightfront_leg.publish(data.rightfront_leg);   
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}