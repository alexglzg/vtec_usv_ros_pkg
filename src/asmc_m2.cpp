#include <iostream>
#include "ros/ros.h"
#include "ros/console.h"
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
float integral_step = 0.01;

int testing = 1;
int arduino = 1;

//Tracking variables
float u_d = 0;
float r_d = 0;

//Auxiliry variables
//float u_line = 0;
//float u_last = 0;
//float u_d_line = 0;
//float u_d_last = 0;
float e_u_int = 0;
float e_u_last = 0;
float e_r_int = 0;
float e_r_last = 0;

//float u_d_dot = 0; surge speed derivative, not necessary

void dspeed_callback(const std_msgs::Float64::ConstPtr& ud)
{
  u_d = ud->data;
}

void dangular_callback(const std_msgs::Float64::ConstPtr& rd)
{
  r_d = rd->data;
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

  ros::init(argc, argv, "asmc_m2");

    ros::NodeHandle n;

  //ROS Publishers for each required sensor data
  ros::Publisher right_thruster_pub = n.advertise<std_msgs::Float64>("right_thruster", 1000);
  ros::Publisher left_thruster_pub = n.advertise<std_msgs::Float64>("left_thruster", 1000);
  ros::Publisher speed_gain_pub = n.advertise<std_msgs::Float64>("speed_gain", 1000);
  ros::Publisher speed_error_pub = n.advertise<std_msgs::Float64>("speed_error", 1000);
  ros::Publisher speed_sigma_pub = n.advertise<std_msgs::Float64>("speed_sigma", 1000);
  ros::Publisher angular_gain_pub = n.advertise<std_msgs::Float64>("angular_gain", 1000);
  ros::Publisher angular_error_pub = n.advertise<std_msgs::Float64>("angular_error", 1000);
  ros::Publisher angular_sigma_pub = n.advertise<std_msgs::Float64>("angular_sigma", 1000);
  ros::Publisher debug_pub = n.advertise<std_msgs::Float64>("debug", 1000);

  ros::Subscriber desired_speed_sub = n.subscribe("desired_speed", 1000, dspeed_callback);
  ros::Subscriber desired_angular_sub = n.subscribe("desired_angular", 1000, dangular_callback);
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
  //float Nrr = 3.5;
  float m = 30;
  float Iz = 4.1;
  float B = 0.41;
  float c = 0.78;

  //Controller gains
  float k_u;
  float kmin_u;
  float k2_u;
  float miu_u;
  float lambda_u;
  float alpha_u;
  float L1_u;

  float k_psi;
  float kmin_psi;
  float k2_psi;
  float miu_psi;
  float lambda_psi;
  float alpha_psi;
  float L1_psi;

  std_msgs::Float64 debug;

  n.getParam("/asmc_m2/k_u", k_u);
  n.getParam("/asmc_m2/kmin_u", kmin_u);
  n.getParam("/asmc_m2/k2_u", k2_u);
  n.getParam("/asmc_m2/mu_u", miu_u);
  n.getParam("/asmc_m2/lambda_u", lambda_u);
  n.getParam("/asmc_m2/alpha_u", alpha_u);
  n.getParam("/asmc_m2/L1_u", L1_u);

  n.getParam("/asmc_m2/k_psi", k_psi);
  n.getParam("/asmc_m2/kmin_psi", kmin_psi);
  n.getParam("/asmc_m2/k2_psi", k2_psi);
  n.getParam("/asmc_m2/mu_psi", miu_psi);
  n.getParam("/asmc_m2/lambda_psi", lambda_psi);
  n.getParam("/asmc_m2/alpha_psi", alpha_psi);
  n.getParam("/asmc_m2/L1_psi", L1_psi);
  


  float Tx = 0;
  float Tz = 0;
  
  float Ka_u = 0;
  float Ka_dot_u = 0;
  float Ka_dot_last_u = 0;
  float ua_u = 0;

  float Ka_psi = 0;
  float Ka_dot_psi = 0;
  float Ka_dot_last_psi = 0;
  float ua_psi = 0;

  
  while (ros::ok()){
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

    //u_line = (0.004)*(u + u_last)/2 + u_line; //integral of the surge speed
    //u_last = u;

    //u_d_line = (0.004)*(u_d + u_d_last)/2 + u_d_line; //integral of the desired surge speed
    //u_d_dot = (u_d - u_d_last) / 0.01; //Derivative, not necessary
    //u_d_last = u_d;

    float e_u = u_d - u;
    float e_r = r_d - r;

    e_u_int = (integral_step)*(e_u + e_u_last)/2 + e_u_int; //integral of the surge speed error
    e_u_last = e_u;
    
    e_r_int = (integral_step)*(e_r + e_r_last)/2 + e_r_int; //integral of the angular speed error
    e_r_last = e_r;

    float sigma_u = e_u + lambda_u * e_u_int;
    float sigma_r = e_r + lambda_psi * e_r_int;
    
    float sigma_u_abs = abs(sigma_u);
    float sigma_r_abs = abs(sigma_r);

//Speed
    if (sigma_u_abs < miu_u){

      int sign_u_sm = 0;

      if (Ka_u > kmin_u){
        float signvar = sigma_u_abs - miu_u;
        if (signvar == 0){
          sign_u_sm = 0;
        }
        else {
          sign_u_sm = copysign(1,signvar);
        }
        Ka_dot_u = k_u * sign_u_sm;
      }
      else{
      Ka_dot_u = kmin_u;
      } 

      Ka_u = (integral_step)*(Ka_dot_u + Ka_dot_last_u)/2 + Ka_u; //integral to get the speed adaptative gain
      Ka_dot_last_u = Ka_dot_u;
    }
    else {
      Ka_u = (1/pow(sigma_u_abs,0.5)) * (alpha_u/pow(2,0.5) + L1_u - k2_u*sigma_u_abs);
      Ka_dot_last_u = 0;
    }

//angular
    if (sigma_r_abs < miu_psi){

      int sign_psi_sm = 0;

      if (Ka_psi > kmin_psi){
        float signvar = sigma_r_abs - miu_psi;
        if (signvar == 0){
          sign_psi_sm = 0;
        }
        else {
          sign_psi_sm = copysign(1,signvar);
        }
        Ka_dot_psi = k_psi * sign_psi_sm;
      }
      else{
      Ka_dot_psi = kmin_psi;
      } 

      Ka_psi = (integral_step)*(Ka_dot_psi + Ka_dot_last_psi)/2 + Ka_psi; //integral to get the angular adaptative gain
      Ka_dot_last_psi = Ka_dot_psi;
      debug.data = 2;
    }
    else {
      Ka_psi = (1/pow(sigma_r_abs,0.5)) * (alpha_psi/pow(2,0.5) + L1_psi - k2_psi*sigma_r_abs);
      Ka_dot_last_psi = 0;
      debug.data = 1;
    }
 
//Speed   
    int sign_u = 0;

    if (sigma_u == 0){
      sign_u = 0;
    }
    else {
      sign_u = copysign(1,sigma_u);
    }
    ua_u = ((-Ka_u) * pow(sigma_u_abs,0.5) * sign_u) - (k2_u*sigma_u);

//angular
    int sign_psi = 0;

    if (sigma_r == 0){
      sign_psi = 0;
    }
    else {
      sign_psi = copysign(1,sigma_r);
    }
    ua_psi = ((-Ka_psi) * pow(sigma_r_abs,0.5) * sign_psi) - (k2_psi*sigma_r);

    Tx = ((lambda_u * e_u) - f_u - ua_u) / g_u; //surge force
    //Tx = 0;
    Tz = ((lambda_psi * e_r) - f_psi - ua_psi) / g_psi; //yaw rate moment
    debug.data = Tz;

    /*if (Tx > 73){
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
    }*/
    
    if (u_d == 0){
      Tx = 0;
      Tz = 0;
      Ka_u = 0;
      Ka_dot_last_u = 0;
      Ka_psi = 0;
      Ka_dot_last_psi = 0;
      e_u_int = 0;
      e_r_int = 0;
      e_u_last = 0;
      e_r_last = 0;
    }

    Tport = (Tx / 2) + (Tz / B);
    Tstbd = (Tx / (2*c)) - (Tz / (B*c));
    
    /*if (Tstbd > 36.5){
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
    }*/

    //Data publishing
    std_msgs::Float64 rt;
    std_msgs::Float64 lt;
    
    std_msgs::Float64 sg;
    std_msgs::Float64 hg;
    std_msgs::Float64 eu;
    std_msgs::Float64 epsi;
    std_msgs::Float64 su;
    std_msgs::Float64 spsi;

    rt.data = Tstbd;
    lt.data = Tport;
    
    sg.data = Ka_u;
    hg.data = Ka_psi;

    eu.data = e_u;
    epsi.data = e_r;

    su.data = sigma_u;
    spsi.data = sigma_r;

    right_thruster_pub.publish(rt);
    left_thruster_pub.publish(lt);

    speed_gain_pub.publish(sg);
    speed_error_pub.publish(eu);
    speed_sigma_pub.publish(su);
    angular_gain_pub.publish(hg);
    angular_error_pub.publish(epsi);
    angular_sigma_pub.publish(spsi);

    debug_pub.publish(debug);
  }
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
