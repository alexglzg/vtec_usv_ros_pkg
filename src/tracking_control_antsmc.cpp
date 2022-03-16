#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"
#include <ros/console.h>

using namespace Eigen;

class AdaptiveSlidingModeControl
{
public:
  //Thruster outputs
  float starboard_t;
  float port_t;

  //Sensor feedback
  float x;
  float y;
  float psi;
  float u;
  float v;
  float r;

  static const float integral_step = 0.01;

  int testing;
  int arduino;
  float starting;

  //Tracking variables
  float x_d;
  float y_d;
  float xdot_d;
  float ydot_d;
  float xddot_d;
  float yddot_d;
  float psi_d;
  float r_d;
  float r_d_dot;
  float past_psi_d;
  float psi_d_dif;
  float past_r_d;

  //Auxiliry variables
  float e_x;
  float e_y;
  float e_x_dot;
  float e_y_dot;
  float xid_x_ddot;
  float xid_y_ddot;
  float s_x;
  float s_y;

  int sign_edx;
  int sign_edy;
  int sign_x_sm;
  int sign_y_sm;
  int sign_sx;
  int sign_sy;

  //Model pysical parameters
  float Xu;
  float Yv;
  float Nr;
  static const float X_u_dot = -2.25;
  static const float Y_v_dot = -23.13;
  static const float N_r_dot = -2.79;
  float Xuu;
  static const float Yvv = -99.99;
  static const float Nrr = -3.49;
  static const float m = 30;
  static const float Iz = 4.1;
  static const float B = 0.41;
  static const float c = 0.78;
  static const float l = 0.5;

  float f_u;
  float g_u;
  float f_v;
  float f_r;
  float g_r;
  
  float Ka_x;
  float Ka_y;
  float Ka_dot_x;
  float Ka_dot_y;
  float Ka_dot_last_x;
  float Ka_dot_last_y;
  float ua_x;
  float ua_y;
  float Tx;
  float Tz;

  //Controller gains
  float k_x;
  float k_y;
  float kmin_x;
  float kmin_y;
  float k2_x;
  float k2_y;
  float mu_x;
  float mu_y;
  float beta_x;
  float beta_y;
  float gamma_x;
  float gamma_y;

  Vector2f u_xi;
  Matrix2f f_1;
  Vector2f f_2;
  Vector2f f_xi;
  Matrix2f g_xi;
  Vector2f lambda_e_xi_dot;
  Vector2f ua_xi;
  Vector2f xid_ddot;

  std_msgs::Float64 right_thruster;
  std_msgs::Float64 left_thruster;

  std_msgs::Float64 x_gain;
  std_msgs::Float64 y_gain;

  std_msgs::Float64 x_error;
  std_msgs::Float64 y_error;

  std_msgs::Float64 x_sigma;
  std_msgs::Float64 y_sigma;

  geometry_msgs::Pose2D ctrl_input;

  AdaptiveSlidingModeControl()
  {
    //ROS Publishers for each required sensor data
    right_thruster_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/right_thruster", 1000);
    left_thruster_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/left_thruster", 1000);
    x_gain_pub = n.advertise<std_msgs::Float64>("/usv_control/asmc/x_gain", 1000);
    x_error_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/x_error", 1000);
    x_sigma_pub = n.advertise<std_msgs::Float64>("/usv_control/asmc/x_sigma", 1000);
    y_sigma_pub = n.advertise<std_msgs::Float64>("/usv_control/asmc/y_sigma", 1000);
    y_gain_pub = n.advertise<std_msgs::Float64>("/usv_control/asmc/y_gain", 1000);
    y_error_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/y_error", 1000);
    control_input_pub = n.advertise<geometry_msgs::Pose2D>("/usv_control/controller/control_input", 1000);

    //ROS Subscribers
    desired_trajectory_sub = n.subscribe("/mission/trajectory", 1000, &AdaptiveSlidingModeControl::desiredTrajCallback, this);
    desired_trajectorydot_sub = n.subscribe("/mission/trajectory_derivative", 1000, &AdaptiveSlidingModeControl::desiredTrajDotCallback, this);
    desired_trajectoryddot_sub = n.subscribe("/mission/trajectory_second_derivative", 1000, &AdaptiveSlidingModeControl::desiredTrajDDotCallback, this);
    ins_pose_sub = n.subscribe("/vectornav/ins_2d/NED_pose", 1000, &AdaptiveSlidingModeControl::insCallback, this);
    local_vel_sub = n.subscribe("/vectornav/ins_2d/local_vel", 1000, &AdaptiveSlidingModeControl::velocityCallback, this);
    flag_sub = n.subscribe("/arduino_br/ardumotors/flag", 1000, &AdaptiveSlidingModeControl::flagCallback, this);
    ardu_sub = n.subscribe("arduino", 1000, &AdaptiveSlidingModeControl::arduinoCallback, this);

    static const float dk_x = 0.05;
    static const float dk_y = 0.05;
    static const float dkmin_x = 0.01;
    static const float dkmin_y = 0.01;
    static const float dk2_x = 0.01;
    static const float dk2_y = 0.01;
    static const float dmu_x = 0.05;
    static const float dmu_y = 0.05;
    static const float dbeta_x = 1.0;
    static const float dbeta_y = 1.0;
    static const float dgamma_x = 1.6;
    static const float dgamma_y = 1.6;

    n.param("/tracking_control_antsmc/k_x", k_x, dk_x);
    n.param("/tracking_control_antsmc/k_y", k_y, dk_y);
    n.param("/tracking_control_antsmc/kmin_x", kmin_x, dkmin_x);
    n.param("/tracking_control_antsmc/kmin_y", kmin_y, dkmin_y);
    n.param("/tracking_control_antsmc/k2_x", k2_x, dk2_x);
    n.param("/tracking_control_antsmc/k2_y", k2_y, dk2_y);
    n.param("/tracking_control_antsmc/mu_x", mu_x, dmu_x);
    n.param("/tracking_control_antsmc/mu_y", mu_y, dmu_y);
    n.param("/tracking_control_antsmc/beta_x", beta_x, dbeta_x);
    n.param("/tracking_control_antsmc/beta_y", beta_y, dbeta_y);
    n.param("/tracking_control_antsmc/gamma_x", gamma_x, dgamma_x);
    n.param("/tracking_control_antsmc/gamma_y", gamma_y, dgamma_y);

    g_u = (1 / (m - X_u_dot));
    g_r = (1 / (Iz - N_r_dot));
    x_d = 0;
    y_d = 0;
    testing = 0;
    arduino = 0;
    starting = 0;
    past_psi_d = 0;
    past_r_d = 0;

  }

  void desiredTrajCallback(const geometry_msgs::Pose2D::ConstPtr& _pd)
  {
    x_d = _pd -> x;
    y_d = _pd -> y;
    starting = _pd -> theta;
  }

  void desiredTrajDotCallback(const geometry_msgs::Pose2D::ConstPtr& _pdotd)
  {
    xdot_d = _pdotd -> x;
    ydot_d = _pdotd -> y;
    psi_d = std::atan2(ydot_d, xdot_d);
  }

  void desiredTrajDDotCallback(const geometry_msgs::Pose2D::ConstPtr& _pddotd)
  {
    xddot_d = _pddotd -> x;
    yddot_d = _pddotd -> y;
  }

  void insCallback(const geometry_msgs::Pose2D::ConstPtr& _ins)
  {
    x = _ins -> x;
    y = _ins -> y;
    psi = _ins -> theta;
  }

  void velocityCallback(const geometry_msgs::Vector3::ConstPtr& _vel)
  {
    u = _vel -> x;
    v = _vel -> y;
    r = _vel -> z;
  }

  void flagCallback(const std_msgs::UInt8::ConstPtr& _flag)
  {
    testing = _flag -> data;
  }

  void arduinoCallback(const std_msgs::UInt8::ConstPtr& _ardu)
  {
    arduino = _ardu -> data;
  }

  void control()
  {
    if (testing == 1 && arduino == 1){
      Xu = -25;
      Xuu = 0;
      if (std::abs(u) > 1.2){
        Xu = 64.55;
        Xuu = -70.92;
      }

      Yv = (-2840)*std::abs(v);
      Nr = (-0.52)*pow(pow(u,2) + pow(v,2),0.5);

      f_u = (((m - Y_v_dot)*v*r + (Xuu*std::abs(u)*u + Xu*u)) / (m - X_u_dot));
      f_v = (((-m + X_u_dot)*u*r + (Yvv*std::abs(v)*v + Yv*v)) / (m - Y_v_dot));
      f_r = (((-X_u_dot + Y_v_dot)*u*v + (Nrr*std::abs(r)*r + Nr*r)) / (Iz - N_r_dot));

      e_x = (x_d + l*cos(psi_d)) - (x + l*cos(psi));
      e_y = (y_d + l*sin(psi_d)) - (y + l*sin(psi));

      psi_d_dif = psi_d - past_psi_d;
      if (std::abs(psi_d_dif) > 3.141592){
          psi_d_dif = (psi_d_dif/std::abs(psi_d_dif))*(std::abs(psi_d_dif) - 2*3.141592);
      }
      r_d = (psi_d_dif)/integral_step;
      e_x_dot = xdot_d - l*sin(psi_d)*r_d - (u*cos(psi) - v*sin(psi) - l*sin(psi)*r);
      e_y_dot = ydot_d + l*cos(psi_d)*r_d - (u*sin(psi) + v*cos(psi) + l*cos(psi)*r);
      past_psi_d = psi_d;

      r_d_dot = (r_d - past_r_d)/integral_step;
      past_r_d = r_d;

      xid_x_ddot = xddot_d - l*sin(psi_d)*r_d_dot - l*cos(psi_d)*r_d*r_d;
      xid_y_ddot = yddot_d + l*cos(psi_d)*r_d_dot - l*sin(psi_d)*r_d*r_d;

      if (e_x_dot == 0){
        sign_edx = 0;
      }
      else {
        sign_edx = copysign(1,e_x_dot);
      }

      if (e_y_dot == 0){
        sign_edy = 0;
      }
      else {
        sign_edy = copysign(1,e_y_dot);
      }

      s_x = e_x + beta_x * pow(std::abs(e_x_dot),gamma_x) * sign_edx;
      s_y = e_y + beta_y * pow(std::abs(e_y_dot),gamma_y) * sign_edy;

      if (Ka_x > kmin_x){
          float signvar = std::abs(s_x) - mu_x;
          if (signvar == 0){
            sign_x_sm = 0;
          }
          else {
            sign_x_sm = copysign(1,signvar);
          }
          Ka_dot_x = k_x * sign_x_sm;
      }
      else{
        Ka_dot_x = kmin_x;
      } 

      Ka_x = (integral_step)*(Ka_dot_x + Ka_dot_last_x)/2 + Ka_x; //integral to get the x adaptative gain
      Ka_dot_last_x = Ka_dot_x;

      if (Ka_y > kmin_y){
          float signvar = std::abs(s_y) - mu_y;
          if (signvar == 0){
            sign_y_sm = 0;
          }
          else {
            sign_y_sm = copysign(1,signvar);
          }
          Ka_dot_y = k_y * sign_y_sm;
      }
      else{
        Ka_dot_y = kmin_y;
      }

      Ka_y = (integral_step)*(Ka_dot_y + Ka_dot_last_y)/2 + Ka_y; //integral to get the y adaptative gain
      Ka_dot_last_y = Ka_dot_y;

      if (s_x == 0){
        sign_sx = 0;
      }
      else {
        sign_sx = copysign(1,s_x);
      }
      ua_x = ((-Ka_x) * pow(std::abs(s_x),0.5) * sign_sx) - (k2_x*s_x);

      if (s_y == 0){
        sign_sy = 0;
      }
      else {
        sign_sy = copysign(1,s_y);
      }
      ua_y = ((-Ka_y) * pow(std::abs(s_y),0.5) * sign_sy) - (k2_y*s_y);

      ua_xi << ua_x,
              ua_y;

      lambda_e_xi_dot << (1.0/(beta_x * gamma_x)) * pow(std::abs(e_x_dot),(2 - gamma_x)) * sign_edx,
                       (1.0/(beta_y * gamma_y)) * pow(std::abs(e_y_dot),(2 - gamma_y)) * sign_edy;

      g_xi << g_u*cos(psi), -g_r*l*sin(psi),
           g_u*sin(psi), g_r*l*cos(psi);

      f_1 << cos(psi), -sin(psi),
           sin(psi), cos(psi);

      f_2 << f_u - v*r - l*r*r,
           u*r + f_v + l*f_r;
      
      f_xi << f_1 * f_2;

      xid_ddot << xid_x_ddot,
                xid_y_ddot;

      u_xi << g_xi.inverse()*(xid_ddot + lambda_e_xi_dot - ua_xi - f_xi);

      Tx = u_xi(0); //surge force
      Tz = u_xi(1); //yaw rate moment
      
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
      
      if (starting == 0){
        Tx = 0;
        Tz = 0;
        Ka_x = kmin_x;
        Ka_dot_last_x = 0;
        Ka_y = kmin_y;
        Ka_dot_last_y = 0;
        past_psi_d = 0;
        past_r_d = 0;
        xddot_d = 0;
        yddot_d = 0;
      }

      port_t = (Tx / 2) + (Tz / B);
      starboard_t = (Tx / (2*c)) - (Tz / (B*c));

      if (starboard_t > 36.5){
        starboard_t = 36.5;
      }
      else if (starboard_t < -30){
        starboard_t = -30;
      }
      if (port_t > 36.5){
        port_t = 36.5;
      }
      else if (port_t < -30){
        port_t = -30;
      }

      //Data publishing
      right_thruster.data = starboard_t;
      left_thruster.data = port_t;

      x_gain.data = Ka_x;
      y_gain.data = Ka_y;

      x_error.data = e_x;
      y_error.data = e_y;

      x_sigma.data = s_x;
      y_sigma.data = s_y;
    
      ctrl_input.x = Tx;
      ctrl_input.theta = Tz;

      right_thruster_pub.publish(right_thruster);
      left_thruster_pub.publish(left_thruster);

      x_gain_pub.publish(x_gain);
      x_error_pub.publish(x_error);
      x_sigma_pub.publish(x_sigma);
      y_gain_pub.publish(y_gain);
      y_error_pub.publish(y_error);
      y_sigma_pub.publish(y_sigma);

      control_input_pub.publish(ctrl_input);
    }
  }

private:
  ros::NodeHandle n;

  ros::Publisher right_thruster_pub;
  ros::Publisher left_thruster_pub;
  ros::Publisher control_input_pub;
  ros::Publisher x_gain_pub;
  ros::Publisher x_error_pub;
  ros::Publisher x_sigma_pub;
  ros::Publisher y_sigma_pub;
  ros::Publisher y_gain_pub;
  ros::Publisher y_error_pub;

  ros::Subscriber desired_trajectory_sub;
  ros::Subscriber desired_trajectorydot_sub;
  ros::Subscriber desired_trajectoryddot_sub;
  ros::Subscriber ins_pose_sub;
  ros::Subscriber local_vel_sub;
  ros::Subscriber flag_sub;
  ros::Subscriber ardu_sub;
};

// Main
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "tracking_control_antsmc");
  AdaptiveSlidingModeControl adaptiveSlidingModeControl;
  int rate = 100;
  ros::Rate loop_rate(rate);

  while (ros::ok())
  {
    adaptiveSlidingModeControl.control();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
