#include "move_pupbot.h"

/* ++++++++++++++++++++++++++++++++++
           Vector2D class
++++++++++++++++++++++++++++++++++ */
Vector2D::Vector2D(double x,double y){
  double_x = x;
  double_y = y;
}

double Vector2D::x(){
  return double_x;
}

double Vector2D::y(){
  return double_y;
}

/* ++++++++++++++++++++++++++++++++++
           Vector class
++++++++++++++++++++++++++++++++++ */
Vector::Vector(double x,double y,double z){
  double_x = x;
  double_y = y;
  double_z = z;
}

double Vector::x(){
  return double_x;
}

double Vector::y(){
  return double_y;
}

double Vector::z(){
  return double_z;
}

/* ++++++++++++++++++++++++++++++++++
            Data class
++++++++++++++++++++++++++++++++++ */
Data::Data(){
  init();
}

void Data::init(){
  pub_leftfront_leg = nh.advertise<trajectory_msgs::JointTrajectory>("/leftfront_leg_controller/command", 10);
  pub_leftback_leg = nh.advertise<trajectory_msgs::JointTrajectory>("/leftback_leg_controller/command", 10);
  pub_rightfront_leg = nh.advertise<trajectory_msgs::JointTrajectory>("/rightfront_leg_controller/command", 10);
  pub_rightback_leg = nh.advertise<trajectory_msgs::JointTrajectory>("/rightback_leg_controller/command", 10);
  key_control_sub1 = nh.subscribe("key_control1",10,&Data::key_controlCallback1,this);
  key_control_sub2 = nh.subscribe("key_control2",10,&Data::startup_shutdown_Callback,this);
  key_control_sub3 = nh.subscribe("key_control3",10,&Data::key_controlCallback2,this);
  x_offset = -14.0;
  z_offset = 117.379725677;
  bone_length = 83.0;
  dirupdate_x = 0.0;
  turn0 = 0.0;
  startup_shutdown_bool=false;
  leftfront_leg.joint_names.resize(3);
  leftfront_leg.points.resize(1);
  leftfront_leg.points[0].positions.resize(3);
  leftfront_leg.joint_names[0] = "leftfront_leg_shoulder_joint";
  leftfront_leg.joint_names[1] = "leftfront_leg_upper_joint";
  leftfront_leg.joint_names[2] = "leftfront_leg_lower_joint";
  leftback_leg.joint_names.resize(3);
  leftback_leg.points.resize(1);
  leftback_leg.points[0].positions.resize(3);
  leftback_leg.joint_names[0] = "leftback_leg_shoulder_joint";
  leftback_leg.joint_names[1] = "leftback_leg_upper_joint";
  leftback_leg.joint_names[2] = "leftback_leg_lower_joint";
  rightfront_leg.joint_names.resize(3);
  rightfront_leg.points.resize(1);
  rightfront_leg.points[0].positions.resize(3);
  rightfront_leg.joint_names[0] = "rightfront_leg_shoulder_joint";
  rightfront_leg.joint_names[1] = "rightfront_leg_upper_joint";
  rightfront_leg.joint_names[2] = "rightfront_leg_lower_joint";
  rightback_leg.joint_names.resize(3);
  rightback_leg.points.resize(1);
  rightback_leg.points[0].positions.resize(3);
  rightback_leg.joint_names[0] = "rightback_leg_shoulder_joint";
  rightback_leg.joint_names[1] = "rightback_leg_upper_joint";
  rightback_leg.joint_names[2] = "rightback_leg_lower_joint";
}

void Data::trot(double c0_x,double c0_y,double dir_x,double dir_y,bool inv,double step_extent_x,double step_extent_y,double step_extent_z,double* vector_x,double* vector_y,double* vector_z){
  double w0 = step_extent_x*0.1/2.0*dir_x;
  double l0 = step_extent_y*0.1*4.0*dir_y;
  double h0 = step_extent_z*0.1;
  if(inv==false){
    c0_x =- c0_x;
    c0_y =- c0_y;
  }
  if(w0 == 0.0 && l0 == 0.0){
    *vector_x = 0.0;
    *vector_y = 0.0;
    *vector_z = 0.0;
  }else if(w0 == 0.0){
    double h1 = sqrt(abs((1.0-(c0_y/l0)*(c0_y/l0))*h0*h0));
    *vector_x = c0_x/0.1;
    *vector_y = c0_y/0.1;
    *vector_z = h1/0.1*int(inv);
  }else if(l0 == 0.0){
    double h1 = sqrt(abs((1.0-(c0_x/w0)*(c0_x/w0))*h0*h0));
    *vector_x = c0_x/0.1;
    *vector_y = c0_y/0.1;
    *vector_z = h1/0.1*int(inv);
  }else{
    double h1 = sqrt(abs((1.0-(c0_x/w0)*(c0_x/w0)-(c0_y/l0)*(c0_y/l0))*h0*h0));
    *vector_x = c0_x/0.1;
    *vector_y = c0_y/0.1;
    *vector_z =h1/0.1*int(inv);
  }
}

void Data::count_c(int l,double dir_x,double dir_y,double step_extent_x,double c_iter[],double c[],double c_inv[]){
  int number = 22;
  double w0 = step_extent_x*0.1*std::max(abs(dir_x),abs(dir_y))/2.0;
  double a0 = (2.0*w0)*(c_iter[l]/number)-w0;
  c[l] = a0;
  c_iter[l] += 1.0;
  if(c_iter[l] > number){
    c[l] = -w0; 
    c_iter[l] = 1.0;
    c_inv[l] += 1.0;
    if (c_inv[l] > 31) c_inv[l] = 0.0;
  }
}

void Data::key_controlCallback1(const std_msgs::Float64& direction_x){
  dirupdate_x=direction_x.data;
}

void Data::key_controlCallback2(const std_msgs::Float64& turn){
  turn0=turn.data;
}

void Data::startup_shutdown_Callback(const std_msgs::Bool& startup_shutdown){
  double startup_shutdown_upper_left = 0.0;
  double startup_shutdown_lower_left = 0.0;
  double startup_shutdown_upper_right = 0.0;
  double startup_shutdown_lower_right = 0.0;
  if(startup_shutdown_bool == false){
    startup_shutdown_upper_left = 1.5;
    startup_shutdown_lower_left = 3.0;
    startup_shutdown_upper_right = -1.5;
    startup_shutdown_lower_right = -3.0;
    for(int i=0;i<22;i++){
      startup_shutdown_upper_left -= 0.68623270987/22.0;
      startup_shutdown_lower_left -= 1.60698754449/22.0;
      startup_shutdown_upper_right += 0.68623270987/22.0;
      startup_shutdown_lower_right += 1.60698754449/22.0;
      rightback_leg.points[0].positions[0] = 0.0;
      rightback_leg.points[0].positions[1] = startup_shutdown_upper_right;
      rightback_leg.points[0].positions[2] = startup_shutdown_lower_right;
      rightback_leg.header.stamp = ros::Time::now();
      rightback_leg.points[0].time_from_start = ros::Duration(0.1);
      pub_rightback_leg.publish(rightback_leg);
      rightfront_leg.points[0].positions[0] = 0.0;
      rightfront_leg.points[0].positions[1] = startup_shutdown_upper_right;
      rightfront_leg.points[0].positions[2] = startup_shutdown_lower_right;
      rightfront_leg.header.stamp = ros::Time::now();
      rightfront_leg.points[0].time_from_start = ros::Duration(0.1);
      pub_rightfront_leg.publish(rightfront_leg); 
      leftfront_leg.points[0].positions[0] = 0.0;
      leftfront_leg.points[0].positions[1] = startup_shutdown_upper_left;
      leftfront_leg.points[0].positions[2] = startup_shutdown_lower_left;
      leftfront_leg.header.stamp = ros::Time::now();
      leftfront_leg.points[0].time_from_start = ros::Duration(0.1);
      pub_leftfront_leg.publish(leftfront_leg);    
      leftback_leg.points[0].positions[0] = 0.0;
      leftback_leg.points[0].positions[1] = startup_shutdown_upper_left;
      leftback_leg.points[0].positions[2] = startup_shutdown_lower_left;
      leftback_leg.header.stamp = ros::Time::now();
      leftback_leg.points[0].time_from_start = ros::Duration(0.1);
      pub_leftback_leg.publish(leftback_leg);
      ros::Duration(0.1).sleep();
    }
    startup_shutdown_bool=true;
  }else if(startup_shutdown_bool == true){
    startup_shutdown_upper_left = 0.81376729013;
    startup_shutdown_lower_left = 1.39301245551;
    startup_shutdown_upper_right = -0.81376729013;
    startup_shutdown_lower_right = -1.39301245551;
    for(int i=0;i<22;i++){
      startup_shutdown_upper_left += 0.68623270987/22.0;
      startup_shutdown_lower_left += 1.60698754449/22.0;
      startup_shutdown_upper_right -= 0.68623270987/22.0;
      startup_shutdown_lower_right -= 1.60698754449/22.0;
      leftfront_leg.points[0].positions[0] = 0.0;
      leftfront_leg.points[0].positions[1] = startup_shutdown_upper_left;
      leftfront_leg.points[0].positions[2] = startup_shutdown_lower_left;
      leftfront_leg.header.stamp = ros::Time::now();
      leftfront_leg.points[0].time_from_start = ros::Duration(0.1);
      pub_leftfront_leg.publish(leftfront_leg);    
      leftback_leg.points[0].positions[0] = 0.0;
      leftback_leg.points[0].positions[1] = startup_shutdown_upper_left;
      leftback_leg.points[0].positions[2] = startup_shutdown_lower_left;
      leftback_leg.header.stamp = ros::Time::now();
      leftback_leg.points[0].time_from_start = ros::Duration(0.1);
      pub_leftback_leg.publish(leftback_leg);
      rightback_leg.points[0].positions[0] = 0.0;
      rightback_leg.points[0].positions[1] = startup_shutdown_upper_right;
      rightback_leg.points[0].positions[2] = startup_shutdown_lower_right;
      rightback_leg.header.stamp = ros::Time::now();
      rightback_leg.points[0].time_from_start = ros::Duration(0.1);
      pub_rightback_leg.publish(rightback_leg);
      rightfront_leg.points[0].positions[0] = 0.0;
      rightfront_leg.points[0].positions[1] = startup_shutdown_upper_right;
      rightfront_leg.points[0].positions[2] = startup_shutdown_lower_right;
      rightfront_leg.header.stamp = ros::Time::now();
      rightfront_leg.points[0].time_from_start = ros::Duration(0.1);
      pub_rightfront_leg.publish(rightfront_leg); 
      ros::Duration(0.1).sleep();
    }
    startup_shutdown_bool=false;
  }
}

double Data::rDir_x(double dir_x,double dir_y){
  if (dir_x != 0.0 || dir_y != 0.0){
    return dir_x/std::max(abs(dir_x),abs(dir_y));
  }else{
    return 1.0;
  } 
}

double Data::rDir_y(double dir_x,double dir_y){
  if (dir_x != 0.0 || dir_y != 0.0){
    return dir_y/std::max(abs(dir_x),abs(dir_y));
  }else{
    return 1.0;
  } 
}

int main(int argc, char** argv){
  ros::init(argc, argv, "move_pupbot");
  Data data;
  ros::Rate loop_rate(50);



  Vector step_extent(40, 40, 5);
  Vector2D direction(0, 0);
  Vector vector(0, 0, 0);
  int l=0;
  double c_iter[4];
  double c[4];
  double c_inv[4];
  double vector_x,vector_y,vector_z;

  while (ros::ok()) {

    for (l=0;l<4;l++) {
      if (data.startup_shutdown_bool == false) continue;
      Vector2D direction(data.dirupdate_x+data.turn0*data.l_inv[l][1],data.turn0*data.l_inv[l][0]);
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
      data.target_left_leg_upper_joint = (90.0-data.angle2)*M_PI/180.0;
      data.target_left_leg_lower_joint = (180.0-data.angle3)*M_PI/180.0;
      data.target_right_leg_upper_joint = -(90.0-data.angle2)*M_PI/180.0;
      data.target_right_leg_lower_joint = -(180.0-data.angle3)*M_PI/180.0;
      ROS_INFO("angle:shoulder=%lf upper=%lf lower=%lf",data.angle1, data.angle2, data.angle3);
      if (l==0) {
        data.leftfront_leg.points[0].positions[0] = data.target_leg_shoulder_joint;
        data.leftfront_leg.points[0].positions[1] = data.target_left_leg_upper_joint;
        data.leftfront_leg.points[0].positions[2] = data.target_left_leg_lower_joint;
        data.leftfront_leg.header.stamp = ros::Time::now();
        data.leftfront_leg.points[0].time_from_start = ros::Duration(0.02);
        data.pub_leftfront_leg.publish(data.leftfront_leg);    
      } else if (l==1) {
        data.leftback_leg.points[0].positions[0] = data.target_leg_shoulder_joint;
        data.leftback_leg.points[0].positions[1] = data.target_left_leg_upper_joint;
        data.leftback_leg.points[0].positions[2] = data.target_left_leg_lower_joint;
        data.leftback_leg.header.stamp = ros::Time::now();
        data.leftback_leg.points[0].time_from_start = ros::Duration(0.02);
        data.pub_leftback_leg.publish(data.leftback_leg);    
      } else if (l==2) {
        data.rightback_leg.points[0].positions[0] = data.target_leg_shoulder_joint;
        data.rightback_leg.points[0].positions[1] = data.target_right_leg_upper_joint;
        data.rightback_leg.points[0].positions[2] = data.target_right_leg_lower_joint;
        data.rightback_leg.header.stamp = ros::Time::now();
        data.rightback_leg.points[0].time_from_start = ros::Duration(0.02);
        data.pub_rightback_leg.publish(data.rightback_leg);    
      } else if (l==3) {
        data.rightfront_leg.points[0].positions[0] = data.target_leg_shoulder_joint;
        data.rightfront_leg.points[0].positions[1] = data.target_right_leg_upper_joint;
        data.rightfront_leg.points[0].positions[2] = data.target_right_leg_lower_joint;
        data.rightfront_leg.header.stamp = ros::Time::now();
        data.rightfront_leg.points[0].time_from_start = ros::Duration(0.02);
        data.pub_rightfront_leg.publish(data.rightfront_leg);   
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}