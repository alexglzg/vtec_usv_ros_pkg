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
  float psi_d;

  //Auxiliry variables
  float e_x;
  float e_y;
  float e_u;
  float e_r;
  float s_x;
  float s_y;
  float s_u;
  float s_r;

  float ei_x;
  float e_x_last;
  float ei_y;
  float e_y_last;
  float ei_u;
  float e_u_last;
  float ei_r;
  float e_r_last;

  int sign_x_sm;
  int sign_y_sm;
  int sign_u_sm;
  int sign_r_sm;
  int sign_sx;
  int sign_sy;
  int sign_su;
  int sign_sr;

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
  //float f_v;
  float f_r;
  float g_r;
  
  float u_u;
  float u_r;
  float k1_x;
  float k1_y;
  float L_x;
  float L_y;
  float L_dot_x;
  float L_dot_y;
  float L_dot_last_x;
  float L_dot_last_y;
  float ua_x;
  float ua_y;
  float k2_x;
  float k2_y;
  float x2_x;
  float x2_y;
  float x2_dot_x;
  float x2_dot_y;
  float x2_dot_last_x;
  float x2_dot_last_y;

  float Tx;
  float Tz;
  float k1_u;
  float k1_r;
  float L_u;
  float L_r;
  float L_dot_u;
  float L_dot_r;
  float L_dot_last_u;
  float L_dot_last_r;
  float ua_u;
  float ua_r;
  float k2_u;
  float k2_r;
  float x2_u;
  float x2_r;
  float x2_dot_u;
  float x2_dot_r;
  float x2_dot_last_u;
  float x2_dot_last_r;

  //Controller gains
  float kmin_x;
  float kmin_y;
  float mu_x;
  float mu_y;
  float lambda_x;
  float lambda_y;

  float kmin_u;
  float kmin_r;
  float mu_u;
  float mu_r;
  float lambda_u;
  float lambda_r;

  Matrix2f Ja;
  Vector2f Ju;
  Vector2f vd;
  Vector2f alpha_eidot_xi;
  Vector2f ua_xi;
  Vector2f xi_dot;

  std_msgs::Float64 right_thruster;
  std_msgs::Float64 left_thruster;
  std_msgs::Float64 u_desired;
  std_msgs::Float64 r_desired;

  std_msgs::Float64 x_gain;
  std_msgs::Float64 y_gain;
  std_msgs::Float64 u_gain;
  std_msgs::Float64 r_gain;

  std_msgs::Float64 x_error;
  std_msgs::Float64 y_error;
  std_msgs::Float64 u_error;
  std_msgs::Float64 r_error;

  std_msgs::Float64 x_sigma;
  std_msgs::Float64 y_sigma;
  std_msgs::Float64 u_sigma;
  std_msgs::Float64 r_sigma;

  geometry_msgs::Pose2D ctrl_input;

  AdaptiveSlidingModeControl()
  {
    //ROS Publishers for each required sensor data
    right_thruster_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/right_thruster", 1000);
    left_thruster_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/left_thruster", 1000);
    speed_gain_pub = n.advertise<std_msgs::Float64>("/usv_control/sast/speed_gain", 1000);
    speed_error_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/speed_error", 1000);
    speed_sigma_pub = n.advertise<std_msgs::Float64>("/usv_control/sast/speed_sigma", 1000);
    heading_sigma_pub = n.advertise<std_msgs::Float64>("/usv_control/sast/heading_sigma", 1000);
    heading_gain_pub = n.advertise<std_msgs::Float64>("/usv_control/sast/heading_gain", 1000);
    heading_error_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/heading_error", 1000);
    control_input_pub = n.advertise<geometry_msgs::Pose2D>("/usv_control/controller/control_input", 1000);
    desired_speed_pub = n.advertise<std_msgs::Float64>("/guidance/desired_speed", 1000);
    desired_heading_pub = n.advertise<std_msgs::Float64>("/guidance/desired_heading", 1000);
    x_gain_pub = n.advertise<std_msgs::Float64>("/usv_control/sast/x_gain", 1000);
    x_error_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/x_error", 1000);
    x_sigma_pub = n.advertise<std_msgs::Float64>("/usv_control/sast/x_sigma", 1000);
    y_sigma_pub = n.advertise<std_msgs::Float64>("/usv_control/sast/y_sigma", 1000);
    y_gain_pub = n.advertise<std_msgs::Float64>("/usv_control/sast/y_gain", 1000);
    y_error_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/y_error", 1000);

    //ROS Subscribers
    desired_trajectory_sub = n.subscribe("/mission/trajectory", 1000, &AdaptiveSlidingModeControl::desiredTrajCallback, this);
    desired_trajectorydot_sub = n.subscribe("/mission/trajectory_derivative", 1000, &AdaptiveSlidingModeControl::desiredTrajDotCallback, this);
    ins_pose_sub = n.subscribe("/vectornav/ins_2d/NED_pose", 1000, &AdaptiveSlidingModeControl::insCallback, this);
    local_vel_sub = n.subscribe("/vectornav/ins_2d/local_vel", 1000, &AdaptiveSlidingModeControl::velocityCallback, this);
    flag_sub = n.subscribe("/arduino_br/ardumotors/flag", 1000, &AdaptiveSlidingModeControl::flagCallback, this);
    ardu_sub = n.subscribe("arduino", 1000, &AdaptiveSlidingModeControl::arduinoCallback, this);

    static const float dkmin_x = 0.01;
    static const float dkmin_y = 0.01;
    static const float dmu_x = 0.05;
    static const float dmu_y = 0.05;
    static const float dlambda_x = 0.001;
    static const float dlambda_y = 0.001;
    static const float dkmin_u = 0.01;
    static const float dkmin_r = 0.01;
    static const float dmu_u = 0.02;
    static const float dmu_r = 0.02;
    static const float dlambda_u = 0.001;
    static const float dlambda_r = 0.001;

    n.param("/tracking_sast2/kmin_x", kmin_x, dkmin_x);
    n.param("/tracking_sast2/kmin_y", kmin_y, dkmin_y);
    n.param("/tracking_sast2/mu_x", mu_x, dmu_x);
    n.param("/tracking_sast2/mu_y", mu_y, dmu_y);
    n.param("/tracking_sast2/lambda_x", lambda_x, dlambda_x);
    n.param("/tracking_sast2/lambda_y", lambda_y, dlambda_y);
    n.param("/tracking_sast2/kmin_u", kmin_u, dkmin_u);
    n.param("/tracking_sast2/kmin_r", kmin_r, dkmin_r);
    n.param("/tracking_sast2/mu_u", mu_u, dmu_u);
    n.param("/tracking_sast2/mu_r", mu_r, dmu_r);
    n.param("/tracking_sast2/lambda_u", lambda_u, dlambda_u);
    n.param("/tracking_sast2/lambda_r", lambda_r, dlambda_r);

    g_u = (1 / (m - X_u_dot));
    g_r = (1 / (Iz - N_r_dot));
    x_d = 0;
    y_d = 0;
    testing = 0;
    arduino = 0;
    starting = 0;

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

      //Yv = (-2840)*std::abs(v);
      Nr = (-0.52)*pow(pow(u,2) + pow(v,2),0.5);

      f_u = (((m - Y_v_dot)*v*r + (Xuu*std::abs(u)*u + Xu*u)) / (m - X_u_dot));
      //f_v = (((-m + X_u_dot)*u*r + (Yvv*std::abs(v)*v + Yv*v)) / (m - Y_v_dot));
      f_r = (((-X_u_dot + Y_v_dot)*u*v + (Nrr*std::abs(r)*r + Nr*r)) / (Iz - N_r_dot));

      e_x = (x_d + l*cos(psi_d)) - (x + l*cos(psi));
      e_y = (y_d + l*sin(psi_d)) - (y + l*sin(psi));

      ei_x = (integral_step)*(e_x + e_x_last)/2 + ei_x; //x integral error
      e_x_last = e_x;
      ei_y = (integral_step)*(e_y + e_y_last)/2 + ei_y; //y integral error
      e_y_last = e_y;

      s_x = e_x + lambda_x*ei_x;
      s_y = e_y + lambda_y*ei_y;

      if (L_x > kmin_x){
        L_dot_x = L_x * (std::abs(s_x) - mu_x);
      }
      else{
        L_dot_x = kmin_x;
      } 

      L_x = (integral_step)*(L_dot_x + L_dot_last_x)/2 + L_x; //integral to get the x adaptative gain
      L_dot_last_x = L_dot_x;

      if (L_y > kmin_y){
        L_dot_y = L_y * (std::abs(s_y) - mu_y);
      }
      else{
        L_dot_y = kmin_y;
      }

      L_y = (integral_step)*(L_dot_y + L_dot_last_y)/2 + L_y; //integral to get the y adaptative gain
      L_dot_last_y = L_dot_y;

      if (s_x == 0){
        sign_sx = 0;
      }
      else {
        sign_sx = copysign(1,s_x);
      }
      k1_x = 2*L_x;
      k2_x = L_x*L_x/2;
      x2_dot_x = -(k2_x) * sign_sx;
      x2_x = (integral_step)*(x2_dot_x + x2_dot_last_x)/2 + x2_x; //integral for x2
      x2_dot_last_x = x2_dot_x;
      ua_x = ((-k1_x) * pow(std::abs(s_x),0.5) * sign_sx) + x2_x;

      if (s_y == 0){
        sign_sy = 0;
      }
      else {
        sign_sy = copysign(1,s_y);
      }
      k1_y = 2*L_y;
      k2_y = L_y*L_y/2;
      x2_dot_y = -(k2_y) * sign_sy;
      x2_y = (integral_step)*(x2_dot_y + x2_dot_last_y)/2 + x2_y; //integral for x2
      x2_dot_last_y = x2_dot_y;
      ua_y = ((-k1_y) * pow(std::abs(s_y),0.5) * sign_sy) + x2_y;

      ua_xi << ua_x,
              ua_y;

      xi_dot << xdot_d,
               ydot_d;

      alpha_eidot_xi << lambda_x * e_x,
                       lambda_y * e_y;

      Ja << cos(psi), -l*sin(psi),
           sin(psi), l*cos(psi);

      Ju << -v*sin(psi),
           v*cos(psi);

      vd << Ja.inverse()*(xi_dot - Ju + alpha_eidot_xi - ua_xi);

      if (vd(0) > 1.4){
        vd(0) = 1.4;
      }
      else if (vd(0) < 0){
        vd(0) = 0;
      }
      if (vd(1) > 1){
        vd(1) = 1;
      }
      else if (vd(1) < -1){
        vd(1) = -1;
      }

      e_u = vd(0) - u;
      e_r = vd(1) - r;

      ei_u = (integral_step)*(e_u + e_u_last)/2 + ei_u; //u integral error
      e_u_last = e_u;
      ei_r = (integral_step)*(e_r + e_r_last)/2 + ei_r; //r integral error
      e_r_last = e_r;

      s_u = e_u + lambda_u*ei_u;
      s_r = e_r + lambda_r*ei_r;

      if (L_u > kmin_u){
        L_dot_u = L_u * (std::abs(s_u) - mu_u);
      }
      else{
        L_dot_u = kmin_u;
      } 

      L_u = (integral_step)*(L_dot_u + L_dot_last_u)/2 + L_u; //integral to get the u adaptative gain
      L_dot_last_u = L_dot_u;

      if (L_r > kmin_r){
        L_dot_r = L_r * (std::abs(s_r) - mu_r);
      }
      else{
        L_dot_r = kmin_r;
      }

      L_r = (integral_step)*(L_dot_r + L_dot_last_r)/2 + L_r; //integral to get the r adaptative gain
      L_dot_last_r = L_dot_r;

      if (s_u == 0){
        sign_su = 0;
      }
      else {
        sign_su = copysign(1,s_u);
      }
      k1_u = 2*L_u;
      k2_u = L_u*L_u/2;
      x2_dot_u = -(k2_u) * sign_su;
      x2_u = (integral_step)*(x2_dot_u + x2_dot_last_u)/2 + x2_u; //integral for x2
      x2_dot_last_u = x2_dot_u;
      ua_u = ((-k1_u) * pow(std::abs(s_u),0.5) * sign_su) + x2_u;

      if (s_r == 0){
        sign_sr = 0;
      }
      else {
        sign_sr = copysign(1,s_r);
      }
      k1_r = 2*L_r;
      k2_r = L_r*L_r/2;
      x2_dot_r = -(k2_r) * sign_sr;
      x2_r = (integral_step)*(x2_dot_r + x2_dot_last_r)/2 + x2_r; //integral for x2
      x2_dot_last_r = x2_dot_r;
      ua_r = ((-L_r) * pow(std::abs(s_r),0.5) * sign_sr) + x2_r;

      Tx = ((lambda_u * e_u) - f_u - ua_u) / g_u; //surge force
      Tz = ((lambda_r * e_r) - f_r - ua_r) / g_r; //yaw rate moment
      
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
        vd(0) = 0;
        vd(1) = 0;
        L_x = kmin_x;
        L_dot_last_x = 0;
        L_y = kmin_y;
        L_dot_last_y = 0;
        L_u = kmin_u;
        L_dot_last_u = 0;
        L_r = kmin_r;
        L_dot_last_r = 0;
        ei_x = 0;
        e_x_last = 0;
        ei_y = 0;
        e_y_last = 0;
        ei_u = 0;
        e_u_last = 0;
        ei_r = 0;
        e_r_last = 0;
        x2_x = 0;
        x2_y = 0;
        x2_dot_last_x = 0;
        x2_dot_last_y = 0;
        x2_u = 0;
        x2_r = 0;
        x2_dot_last_u = 0;
        x2_dot_last_r = 0;
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
      u_desired.data = vd(0);
      r_desired.data = vd(1);
      
      x_gain.data = k1_x;
      y_gain.data = k1_y;
      u_gain.data = k1_u;
      r_gain.data = k1_r;

      x_error.data = e_x;
      y_error.data = e_y;
      u_error.data = e_u;
      r_error.data = e_r;

      x_sigma.data = s_x;
      y_sigma.data = s_y;
      u_sigma.data = s_u;
      r_sigma.data = s_r;
    
      ctrl_input.x = Tx;
      ctrl_input.theta = Tz;

      right_thruster_pub.publish(right_thruster);
      left_thruster_pub.publish(left_thruster);
      desired_speed_pub.publish(u_desired);
      desired_heading_pub.publish(r_desired);

      speed_gain_pub.publish(u_gain);
      speed_error_pub.publish(u_error);
      speed_sigma_pub.publish(u_sigma);
      heading_gain_pub.publish(r_gain);
      heading_error_pub.publish(r_error);
      heading_sigma_pub.publish(r_sigma);
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
  ros::Publisher speed_gain_pub;
  ros::Publisher speed_error_pub;
  ros::Publisher speed_sigma_pub;
  ros::Publisher heading_sigma_pub;
  ros::Publisher heading_gain_pub;
  ros::Publisher heading_error_pub;
  ros::Publisher control_input_pub;
  ros::Publisher x_gain_pub;
  ros::Publisher x_error_pub;
  ros::Publisher x_sigma_pub;
  ros::Publisher y_sigma_pub;
  ros::Publisher y_gain_pub;
  ros::Publisher y_error_pub;
  ros::Publisher desired_speed_pub;
  ros::Publisher desired_heading_pub;

  ros::Subscriber desired_trajectory_sub;
  ros::Subscriber desired_trajectorydot_sub;
  ros::Subscriber ins_pose_sub;
  ros::Subscriber local_vel_sub;
  ros::Subscriber flag_sub;
  ros::Subscriber ardu_sub;
};

// Main
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "tracking_sast2");
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