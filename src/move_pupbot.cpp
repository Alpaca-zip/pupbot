#include "move_pupbot.h"

/* ++++++++++++++++++++++++++++++++++
         Move_Pupbot class
++++++++++++++++++++++++++++++++++ */
Move_Pupbot::Move_Pupbot(){
  init();
}

void Move_Pupbot::init(){
  number = 22;
  x_offset = X_OFFSET;
  z_offset = Z_OFFSET;
  bone_length = BONE_LENGTH;
  dirupdate_x = 0.0;
  turn0 = 0.0;
  step_extent_x = STEP_EXTENT_X;
  step_extent_y = STEP_EXTENT_Y;
  step_extent_z = STEP_EXTENT_Z;
  startup_shutdown_bool=false;
  pub_leftfront_leg = nh.advertise<trajectory_msgs::JointTrajectory>("/leftfront_leg_controller/command", 10);
  pub_leftback_leg = nh.advertise<trajectory_msgs::JointTrajectory>("/leftback_leg_controller/command", 10);
  pub_rightfront_leg = nh.advertise<trajectory_msgs::JointTrajectory>("/rightfront_leg_controller/command", 10);
  pub_rightback_leg = nh.advertise<trajectory_msgs::JointTrajectory>("/rightback_leg_controller/command", 10);
  key_control_sub1 = nh.subscribe("key_control1", 10, &Move_Pupbot::key_controlCallback1, this);
  key_control_sub2 = nh.subscribe("key_control2", 10, &Move_Pupbot::startup_shutdown_Callback, this);
  key_control_sub3 = nh.subscribe("key_control3", 10, &Move_Pupbot::key_controlCallback2, this);
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

void Move_Pupbot::trot(double c0_x, double c0_y, bool inv){
  w0 = step_extent_x*0.1/2.0*dir_x;
  l0 = step_extent_y*0.1*4.0*dir_y;
  h0 = step_extent_z*0.1;
  if(inv == false){
    c0_x =- c0_x;
    c0_y =- c0_y;
  }
  if(w0 == 0.0 && l0 == 0.0){
    vector_x = 0.0;
    vector_y = 0.0;
    vector_z = 0.0;
  }else if(w0 == 0.0){
    double h1 = sqrt(abs((1.0-(c0_y/l0)*(c0_y/l0))*h0*h0));
    vector_x = c0_x/0.1;
    vector_y = c0_y/0.1;
    vector_z = h1/0.1*int(inv);
  }else if(l0 == 0.0){
    double h1 = sqrt(abs((1.0-(c0_x/w0)*(c0_x/w0))*h0*h0));
    vector_x = c0_x/0.1;
    vector_y = c0_y/0.1;
    vector_z = h1/0.1*int(inv);
  }else{
    double h1 = sqrt(abs((1.0-(c0_x/w0)*(c0_x/w0)-(c0_y/l0)*(c0_y/l0))*h0*h0));
    vector_x = c0_x/0.1;
    vector_y = c0_y/0.1;
    vector_z = h1/0.1*int(inv);
  }
}

void Move_Pupbot::count_c(){
  w0_count_c = step_extent_x*0.1*std::max(abs(dir_x),abs(dir_y))/2.0;
  a0_count_c = (2.0*w0_count_c)*(c_iter[l]/number)-w0_count_c;
  c[l] = a0_count_c;
  c_iter[l] += 1.0;
  if(c_iter[l] > number){
    c[l] = -w0_count_c;
    c_iter[l] = 1.0;
    c_inv[l] += 1.0;
    if (c_inv[l] > 31)c_inv[l] = 0.0;
  }
}

void Move_Pupbot::key_controlCallback1(const std_msgs::Float64& direction_x){
  dirupdate_x = direction_x.data;
}

void Move_Pupbot::key_controlCallback2(const std_msgs::Float64& turn){
  turn0 = turn.data;
}

void Move_Pupbot::startup_shutdown_Callback(const std_msgs::Bool& startup_shutdown){
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
    startup_shutdown_bool = true;
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
    startup_shutdown_bool = false;
  }
}

void Move_Pupbot::controlLoop(){
  for(l=0;l<4;l++){
    if(startup_shutdown_bool == false)continue;
    dir_x = dirupdate_x+turn0*l_inv[l][1];
    dir_y = turn0*l_inv[l][0];
    count_c();
    trot(rDir_x()*c[l], l_inv[l][1]*rDir_y()*c[l], bool(l%2)^bool(fmod(c_inv[l], 2.0)));
    x = x_offset+vector_x;
    y = vector_y;
    z = z_offset-vector_z;
    a0 = (atan(x/z))*180.0/M_PI;
    a1 = (atan(y/z))*180.0/M_PI;
    b0 = sqrt(x*x+z*z);
    angle1 = a1;
    angle3 = (asin((b0/2.0)/bone_length))*180.0/M_PI*2.0;
    angle2 = angle3/2.0+a0;
    target_leg_shoulder_joint = angle1*M_PI/180.0;
    target_left_leg_upper_joint = (90.0-angle2)*M_PI/180.0;
    target_left_leg_lower_joint = (180.0-angle3)*M_PI/180.0;
    target_right_leg_upper_joint = -(90.0-angle2)*M_PI/180.0;
    target_right_leg_lower_joint = -(180.0-angle3)*M_PI/180.0;
    ROS_INFO("angle:shoulder=%lf upper=%lf lower=%lf", angle1,angle2,angle3);
    if(l == 0){
      leftfront_leg.points[0].positions[0] = target_leg_shoulder_joint;
      leftfront_leg.points[0].positions[1] = target_left_leg_upper_joint;
      leftfront_leg.points[0].positions[2] = target_left_leg_lower_joint;
      leftfront_leg.header.stamp = ros::Time::now();
      leftfront_leg.points[0].time_from_start = ros::Duration(0.02);
      pub_leftfront_leg.publish(leftfront_leg);    
    }else if(l == 1){
      leftback_leg.points[0].positions[0] = target_leg_shoulder_joint;
      leftback_leg.points[0].positions[1] = target_left_leg_upper_joint;
      leftback_leg.points[0].positions[2] = target_left_leg_lower_joint;
      leftback_leg.header.stamp = ros::Time::now();
      leftback_leg.points[0].time_from_start = ros::Duration(0.02);
      pub_leftback_leg.publish(leftback_leg);    
    }else if(l == 2){
      rightback_leg.points[0].positions[0] = target_leg_shoulder_joint;
      rightback_leg.points[0].positions[1] = target_right_leg_upper_joint;
      rightback_leg.points[0].positions[2] = target_right_leg_lower_joint;
      rightback_leg.header.stamp = ros::Time::now();
      rightback_leg.points[0].time_from_start = ros::Duration(0.02);
      pub_rightback_leg.publish(rightback_leg);    
    }else if(l == 3){
      rightfront_leg.points[0].positions[0] = target_leg_shoulder_joint;
      rightfront_leg.points[0].positions[1] = target_right_leg_upper_joint;
      rightfront_leg.points[0].positions[2] = target_right_leg_lower_joint;
      rightfront_leg.header.stamp = ros::Time::now();
      rightfront_leg.points[0].time_from_start = ros::Duration(0.02);
      pub_rightfront_leg.publish(rightfront_leg);   
    }
  }
}

double Move_Pupbot::rDir_x(){
  if(dir_x != 0.0 || dir_y != 0.0){
    return dir_x/std::max(abs(dir_x), abs(dir_y));
  }else{
    return 1.0;
  } 
}

double Move_Pupbot::rDir_y(){
  if(dir_x != 0.0 || dir_y != 0.0){
    return dir_y/std::max(abs(dir_x), abs(dir_y));
  }else{
    return 1.0;
  } 
}

/* ++++++++++++++++++++++++++++++++++
               main
++++++++++++++++++++++++++++++++++ */
int main(int argc, char** argv){
  ros::init(argc,argv, "move_pupbot");
  Move_Pupbot PupBot;

  ros::Rate loop_rate(50);

  while(ros::ok()){
    PupBot.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}