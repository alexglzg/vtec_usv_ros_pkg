#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"
#include <math.h>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

//Thruster outputs
float Tstbd = 0;
float Tport = 0;

//Sensor feedback
float theta = 0;
float u = 0;
float v = 0;
float r = 0;

int rate = 100;
float time_step = 0.01;

int testing = 0;
int arduino = 0;

//Tracking variables
float u_d = 0;
float psi_d = 0;

//Auxiliry variables
//float u_line = 0;
//float u_last = 0;
//float u_d_line = 0;
//float u_d_last = 0;
float e_u_int = 0;
float e_u_dot = 0;
float e_u_last = 0;
//float e_psi_int = 0;
//float e_psi_last = 0;

//float u_d_dot = 0; surge speed derivative, not necessary

void dspeed_callback(const std_msgs::Float64::ConstPtr& ud)
{
  u_d = ud->data;
}

void dheading_callback(const std_msgs::Float64::ConstPtr& psid)
{
  psi_d = psid->data;
}

void ins_callback(const geometry_msgs::Pose2D::ConstPtr& ins)
{
  theta = ins->theta;
}

void vel_callback(const geometry_msgs::Vector3::ConstPtr& vel)
{
  u = vel->x;
  v = vel->y; 
  r = vel->z;
}

void flag_callback(const std_msgs::UInt8::ConstPtr& flag)
{
  testing = flag->data;
}

void ardu_callback(const std_msgs::UInt8::ConstPtr& ardu)
{
  arduino = ardu->data;
}

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "pid");

    ros::NodeHandle n;

  //ROS Publishers for each required sensor data
  ros::Publisher right_thruster_pub = n.advertise<std_msgs::Float64>("right_thruster", 1000);
  ros::Publisher left_thruster_pub = n.advertise<std_msgs::Float64>("left_thruster", 1000);
  ros::Publisher speed_error_pub = n.advertise<std_msgs::Float64>("speed_error", 1000);
  ros::Publisher heading_error_pub = n.advertise<std_msgs::Float64>("heading_error", 1000);

  ros::Subscriber desired_speed_sub = n.subscribe("desired_speed", 1000, dspeed_callback);
  ros::Subscriber desired_heading_sub = n.subscribe("desired_heading", 1000, dheading_callback);
  ros::Subscriber ins_pose_sub = n.subscribe("ins_pose", 1000, ins_callback);
  ros::Subscriber local_vel_sub = n.subscribe("local_vel", 1000, vel_callback);
  ros::Subscriber flag_sub = n.subscribe("flag", 1000, flag_callback);
  ros::Subscriber ardu_sub = n.subscribe("arduino", 1000, ardu_callback);

  ros::Rate loop_rate(rate);

  //Model pysical parameters
  float Xu = 0;
  float Nr = 0;
  float X_u_dot = -2.25;
  float Y_v_dot = -23.13;
  float N_r_dot = -2.79;
  float Xuu = 0;
  float m = 30;
  float Iz = 4.1;
  float B = 0.41;
  float c = 0.78;

  //Controller gains
  float kp_u = 1.1;
  float kd_u = 0.1;
  float ki_u = 0.2;
  float kp_psi = 0.8;
  float kd_psi = 3;
    
  float Tx = 0;
  float Tz = 0;
  float ua_u = 0;
  float ua_psi = 0;

  while (ros::ok())
  {
  if (testing == 1 && arduino == 1){
    Xu = -25;
    Xuu = 0;
    float u_abs = abs(u);
    if (u_abs > 1.2){
      Xu = 64.55;
      Xuu = -70.92;
    }

    Nr = (-0.52)*pow(pow(u,2)+pow(v,2),0.5);

    float g_u = (1 / (m - X_u_dot));
    float g_psi = (1 / (Iz - N_r_dot));

    float f_u = (((m - Y_v_dot)*v*r + (Xuu*u_abs*u + Xu*u)) / (m - X_u_dot));
    float f_psi = (((-X_u_dot + Y_v_dot)*u*v + (Nr*r)) / (Iz - N_r_dot));

    float e_u = u_d - u;
    e_u_int = (time_step)*(e_u + e_u_last)/2 + e_u_int; //integral of the surge speed error
    e_u_dot = (e_u - e_u_last) / time_step; //derivate of the surge speed error
    e_u_last = e_u;
    
    float e_psi = psi_d - theta;
    if (abs(e_psi) > 3.141592){
        e_psi = (e_psi/abs(e_psi))*(abs(e_psi)-2*3.141592);
    }
    float e_psi_dot = 0 - r; //derivate of the heading error

    ua_u = (kp_u * e_u) + (ki_u * e_u_int) + (kd_u * e_u_dot);
    ua_psi = (kp_psi * e_psi) + (kd_psi * e_psi_dot);
  
    Tx = (-f_u + ua_u) / g_u; //surge force
    Tz = (-f_psi + ua_psi) / g_psi; //yaw rate moment
    
    if (Tx > 73){
      Tx = 73;
    }
    else if (Tx < -60){
      Tx = -60;
    }
    if (Tz > 14){
      Tz = 14;
    }
    else if (Tz < -14){
      Tz = -14;
    }
    
    if (u_d == 0){
      Tx = 0;
      Tz = 0;
      e_u_int = 0;
      e_u_last = 0;
    }

    Tport = (Tx / 2) + (Tz / B);
    Tstbd = (Tx / (2*c)) - (Tz / (B*c));

    
    if (Tstbd > 36.5){
      Tstbd = 36.5;
    }
    else if (Tstbd < -30){
      Tstbd = -30;
    }
    if (Tport > 36.5){
      Tport = 36.5;
    }
    else if (Tport < -30){
      Tport = -30;
    }
    
    //Data publishing
    std_msgs::Float64 rt;
    std_msgs::Float64 lt;
    
    std_msgs::Float64 eu;
    std_msgs::Float64 epsi;

    rt.data = Tstbd;
    lt.data = Tport;
    
    eu.data = e_u;
    epsi.data = e_psi;

    right_thruster_pub.publish(rt);
    left_thruster_pub.publish(lt);

    speed_error_pub.publish(eu);
    heading_error_pub.publish(epsi);
  }
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
