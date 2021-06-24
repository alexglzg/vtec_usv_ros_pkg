#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"


class AdaptiveSlidingModeControl
{
public:
  //Thruster outputs
  float starboard_t;
  float port_t;

  //Sensor feedback
  float u;
  float v;
  float r;

  static const float integral_step = 0.01;

  int testing;
  int arduino;
  int start;
  double start_time;
  double convergence_time_u;
  double convergence_time_r;

  //Tracking variables
  float u_d;
  float r_d;

  //Auxiliry variables
  float e_u;
  float e_u_int;
  float e_u_last;
  float e_r;
  float e_r_int;
  float e_r_last;
  float sigma_u;
  float sigma_u_last;
  float sigma_u_dot;
  float sigma_u_abs;
  float sigma_r;
  float sigma_r_last;
  float sigma_r_dot;
  float sigma_r_abs;
  float Delta_hat_dot_last_u;
  float Delta_hat_dot_last_r;
  float sqr2;
  int convergence_u;
  int convergence_r;
  float u_abs;
  float g_u;
  float g_r;
  float f_u;
  float f_r;
  int sign_u;
  int sign_r;
  int sign_delta_tilde_u;
  int sign_delta_tilde_r;

  float o_dot_dot;
  float o_dot;
  float o;
  float o_last;
  float o_dot_last;
  float o_dot_dot_last;
  static const float o1 = 2;
  static const float o2 = 2;
  static const float o3 = 2;

  float g_dot_dot;
  float g_dot;
  float g;
  float g_last;
  float g_dot_last;
  float g_dot_dot_last;
  static const float g1 = 2;
  static const float g2 = 2;
  static const float g3 = 2;

  //Model pysical parameters
  float Xu;
  float Nr;
  static const float X_u_dot = -2.25;
  static const float Y_v_dot = -23.13;
  static const float N_r_dot = -2.79;
  float Xuu;
  static const float Nrr = -3.49;
  static const float m = 30;
  static const float Iz = 4.1;
  static const float B = 0.41;
  static const float c = 0.78;
  
  float Tx;
  float Tz;
  float Ka_u;
  float Ka_r;
  float Ka_dot_u;
  float Ka_dot_r;
  float Ka_dot_last_u;
  float Ka_dot_last_r;
  float ua_u;
  float ua_r;

  //Controller gains
  float K2_u;
  float K2_r;
  float mu_u;
  float mu_r;
  float lambda_u;
  float lambda_r;
  float eta_u;
  float eta_r;
  float k_u;
  float k_r;
  float Kmin_u;
  float Kmin_r;

  //Fixed-time variables
  float L1_u;
  float L1_dot_u;
  float L1_r;
  float L1_dot_r;
  float epsilon;
  float ro;
  float t_c;

  //Observer
  float Delta_hat_u;
  float Delta_hat_r;
  float Delta_tilde_u;
  float Delta_tilde_r;
  float Delta_hat_dot_u;
  float Delta_hat_dot_r;

  float k_delta_u;
  float k_delta_r;

  AdaptiveSlidingModeControl()
  {
    //ROS Publishers for each required sensor data
    right_thruster_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/right_thruster", 1000);
    left_thruster_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/left_thruster", 1000);
    speed_gain_pub = n.advertise<std_msgs::Float64>("/usv_control/asmc/speed_gain", 1000);
    speed_error_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/speed_error", 1000);
    speed_sigma_pub = n.advertise<std_msgs::Float64>("/usv_control/asmc/speed_sigma", 1000);
    heading_sigma_pub = n.advertise<std_msgs::Float64>("/usv_control/asmc/heading_sigma", 1000);
    heading_gain_pub = n.advertise<std_msgs::Float64>("/usv_control/asmc/heading_gain", 1000);
    heading_error_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/heading_error", 1000);
    control_input_pub = n.advertise<geometry_msgs::Pose2D>("/usv_control/controller/control_input", 1000);
    observer_pub = n.advertise<geometry_msgs::Pose2D>("/usv_control/observer", 1000);
    observer_error_pub = n.advertise<geometry_msgs::Pose2D>("/usv_control/observer_error", 1000);
    
    //ROS Subscribers
    desired_speed_sub = n.subscribe("/guidance/desired_speed", 1000, &AdaptiveSlidingModeControl::desiredSpeedCallback, this);
    desired_heading_sub = n.subscribe("/guidance/desired_heading", 1000, &AdaptiveSlidingModeControl::desiredHeadingCallback, this);
    local_vel_sub = n.subscribe("/vectornav/ins_2d/local_vel", 1000, &AdaptiveSlidingModeControl::velocityCallback, this);
    flag_sub = n.subscribe("/arduino_br/ardumotors/flag", 1000, &AdaptiveSlidingModeControl::flagCallback, this);
    ardu_sub = n.subscribe("arduino", 1000, &AdaptiveSlidingModeControl::arduinoCallback, this);
    start_sub = n.subscribe("/start_prescribed", 1000, &AdaptiveSlidingModeControl::startCallback, this);

    static const float dL1_u = 0.0;
    static const float dL1_dot_u = 0.0;
    static const float dL1_r = 0.0;
    static const float dL1_dot_r = 0.0;
    static const float depsilon = 0.001;
    static const float dro = 0.1;
    static const float dK2_u = 0.1;
    static const float dK2_r = 0.1;
    static const float dmu_u = 0.05;
    static const float dmu_r = 0.05;
    static const float dlambda_u = 0.01;
    static const float dlambda_r = 0.01;
    static const float dk_u = 0.1;
    static const float dk_r = 0.1;
    static const float dKmin_u = 0.05;
    static const float dKmin_r = 0.05;
    static const float dt_c = 3.0;

    n.param("/obs_finite_time_asmc/L1_u", L1_u, dL1_u);
    n.param("/obs_finite_time_asmc/L1_dot_u", L1_dot_u, dL1_dot_u);
    n.param("/obs_finite_time_asmc/L1_r", L1_r, dL1_r);
    n.param("/obs_finite_time_asmc/L1_dot_r", L1_dot_r, dL1_dot_r);
    n.param("/obs_finite_time_asmc/epsilon", epsilon, depsilon);
    n.param("/obs_finite_time_asmc/ro", ro, dro);
    n.param("/obs_finite_time_asmc/K2_u", K2_u, dK2_u);
    n.param("/obs_finite_time_asmc/K2_r", K2_r, dK2_r);
    n.param("/obs_finite_time_asmc/mu_u", mu_u, dmu_u);
    n.param("/obs_finite_time_asmc/mu_r", mu_r, dmu_r);
    n.param("/obs_finite_time_asmc/lambda_u", lambda_u, dlambda_u);
    n.param("/obs_finite_time_asmc/lambda_r", lambda_r, dlambda_r);
    n.param("/obs_finite_time_asmc/k_u", k_u, dk_u);
    n.param("/obs_finite_time_asmc/k_r", k_r, dk_r);
    n.param("/obs_finite_time_asmc/Kmin_u", Kmin_u, dKmin_u);
    n.param("/obs_finite_time_asmc/Kmin_r", Kmin_r, dKmin_r);
    n.param("/obs_finite_time_asmc/t_c", t_c, dt_c);

    u_d = 0;
    r_d = 0;
    testing = 0;
    arduino = 0;
    sqr2 = std::pow(2,0.5);
    Delta_hat_u = 0;
    Delta_hat_r = 0;
    convergence_u = 0;
    convergence_r = 0;
    start = 0;

  }

  void desiredSpeedCallback(const std_msgs::Float64::ConstPtr& _ud)
  {
    u_d = _ud -> data;
  }

  void desiredHeadingCallback(const std_msgs::Float64::ConstPtr& _rd)
  {
    r_d = _rd -> data;
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

  void startCallback(const std_msgs::UInt8::ConstPtr& _start)
  {
    start = _start -> data;
  }

  void control()
  {
    if (testing == 1 && arduino == 1){
      Xu = -25;
      Xuu = 0;
      u_abs = std::abs(u);
      if (u_abs > 1.2){
        Xu = 64.55;
        Xuu = -70.92;
      }

      Nr = (-0.52)*std::pow(std::pow(u,2) + std::pow(v,2),0.5);

      g_u = (1 / (m - X_u_dot));
      g_r = (1 / (Iz - N_r_dot));

      f_u = (((m - Y_v_dot)*v*r + (Xuu*u_abs*u + Xu*u)) / (m - X_u_dot));
      f_r = (((-X_u_dot + Y_v_dot)*u*v + (Nr*r + Nrr*r*std::abs(r))) / (Iz - N_r_dot));

      e_u = u_d - u;
      e_r = r_d - r;

      e_u_int = (integral_step)*(e_u + e_u_last)/2 + e_u_int; //integral of the surge speed error
      e_u_last = e_u;

      e_r_int = (integral_step)*(e_r + e_r_last)/2 + e_r_int; //integral of the heading error
      e_r_last = e_r;

      sigma_u = e_u + lambda_u * e_u_int;
      sigma_r = e_r + lambda_r * e_r_int;

      sigma_u_dot = (sigma_u - sigma_u_last) / integral_step;

      o_dot_dot = (((sigma_u_dot - o_last) * o1) - (o3 * o_dot_last)) * o2;
      o_dot = (integral_step)*(o_dot_dot + o_dot_dot_last)/2 + o_dot;
      o = (integral_step)*(o_dot + o_dot_last)/2 + o;
      sigma_u_dot = o;
      o_last = o;
      o_dot_last = o_dot;
      o_dot_dot_last = o_dot_dot;
      sigma_u_last = sigma_u;

      sigma_r_dot = (sigma_r - sigma_r_last) / integral_step;

      g_dot_dot = (((sigma_r_dot - g_last) * g1) - (g3 * g_dot_last)) * g2;
      g_dot = (integral_step)*(g_dot_dot + g_dot_dot_last)/2 + g_dot;
      g = (integral_step)*(g_dot + g_dot_last)/2 + g;
      sigma_r_dot = g;
      g_last = g;
      g_dot_last = g_dot;
      g_dot_dot_last = g_dot_dot;
      sigma_r_last = sigma_r;
      
      sigma_u_abs = std::abs(sigma_u);
      sigma_r_abs = std::abs(sigma_r);

      if (start == 1){
        eta_u = sqr2*sigma_u_abs / t_c;
        eta_r = sqr2*sigma_r_abs / t_c;
        ROS_FATAL_STREAM("eta_u = " << eta_u);
        ROS_FATAL_STREAM("eta_r = " << eta_r);
        start_time = ros::Time::now().toSec();
      }

      if (((sigma_u_abs <= mu_u) || (ros::Time::now().toSec() - start_time >= t_c)) && ((convergence_u == 0)  && (start > 0))){
        convergence_u = 1;
        convergence_time_u = ros::Time::now().toSec() - start_time;
        ROS_FATAL_STREAM("convergence time u = " << convergence_time_u);
      }

      if (((sigma_r_abs <= mu_r) || (ros::Time::now().toSec() - start_time >= t_c)) && ((convergence_r == 0)  && (start > 0))){
        convergence_r = 1;
        convergence_time_r = ros::Time::now().toSec() - start_time;
        ROS_FATAL_STREAM("convergence time r = " << convergence_time_r);
      }

      if (sigma_u == 0){
        sign_u = 0;
      }
      else {
        sign_u = copysign(1,sigma_u);
      }

      if (sigma_r == 0){
        sign_r = 0;
      }
      else {
        sign_r = copysign(1,sigma_r);
      }

      if (convergence_u == 0){
        Ka_u = (1/std::pow(sigma_u_abs,0.5)) * (Delta_hat_u*sign_u + (eta_u/sqr2) - K2_u*sigma_u_abs);
      }
      else if (convergence_u == 1){
        int sign_u_sm = 0;
        if (Ka_u > Kmin_u){
          float signvar = sigma_u_abs - mu_u;
          if (signvar == 0){
            sign_u_sm = 0;
          }
          else {
            sign_u_sm = copysign(1,signvar);
          }
          Ka_dot_u = k_u * sign_u_sm;
        }
        else{
          Ka_dot_u = Kmin_u;
        }
        Ka_u = (integral_step)*(Ka_dot_u + Ka_dot_last_u)/2 + Ka_u;
        Ka_dot_last_u = Ka_dot_u; 
      }

      if (convergence_r == 0){
        Ka_r = (1/std::pow(sigma_r_abs,0.5)) * (Delta_hat_r*sign_r + (eta_r/sqr2) - (K2_r*sigma_r_abs));
      }
      else if (convergence_r == 1){
        int sign_r_sm = 0;
        if (Ka_r > Kmin_r){
          float signvar = sigma_r_abs - mu_r;
          if (signvar == 0){
            sign_r_sm = 0;
          }
          else {
            sign_r_sm = copysign(1,signvar);
          }
          Ka_dot_r = k_r * sign_r_sm;
        }
        else{
          Ka_dot_r = Kmin_r;
        }
        Ka_r = (integral_step)*(Ka_dot_r + Ka_dot_last_r)/2 + Ka_r;
        Ka_dot_last_r = Ka_dot_r;
      }

      ua_u = ((-Ka_u) * std::pow(sigma_u_abs,0.5) * sign_u) - (K2_u*sigma_u);
      ua_r = ((-Ka_r) * std::pow(sigma_r_abs,0.5) * sign_r) - (K2_r*sigma_r);

      Delta_tilde_u = sigma_u_dot - Delta_hat_u - ua_u;
      Delta_tilde_r = sigma_r_dot - Delta_hat_r - ua_r;

      sign_delta_tilde_u = 0;
      sign_delta_tilde_r = 0;

      if (Delta_tilde_u == 0){
        sign_delta_tilde_u = 0;
      }
      else {
        sign_delta_tilde_u = copysign(1,Delta_tilde_u);
      }

      if (Delta_tilde_r == 0){
        sign_delta_tilde_r = 0;
      }
      else {
        sign_delta_tilde_r = copysign(1,Delta_tilde_r);
      }

      if (std::abs(Delta_tilde_u) < epsilon){
        k_delta_u = L1_dot_u / std::pow(std::abs(epsilon),0.5);
      }
      else{
        k_delta_u = (1 / std::pow(std::abs(Delta_tilde_u),0.5)) * ((L1_u * eta_u)/(ro*std::abs(u_d)*sqr2) + L1_dot_u);
      }

      if (std::abs(Delta_tilde_r) < epsilon){
        k_delta_r = L1_dot_r / std::pow(std::abs(epsilon),0.5);
      }
      else{
        k_delta_r = (1 / std::pow(std::abs(Delta_tilde_r),0.5)) * ((L1_r * eta_r)/(ro*std::abs(r_d)*sqr2) + L1_dot_r);
      }

      Delta_hat_dot_u = k_delta_u * std::pow(std::abs(Delta_tilde_u),0.5) * sign_delta_tilde_u;
      Delta_hat_dot_r = k_delta_r * std::pow(std::abs(Delta_tilde_r),0.5) * sign_delta_tilde_r;

      Delta_hat_u = (integral_step)*(Delta_hat_dot_u + Delta_hat_dot_last_u)/2 + Delta_hat_u;
      Delta_hat_dot_last_u = Delta_hat_dot_u;

      Delta_hat_r = (integral_step)*(Delta_hat_dot_r + Delta_hat_dot_last_r)/2 + Delta_hat_r;
      Delta_hat_dot_last_r = Delta_hat_dot_r;

      Tx = ((lambda_u * e_u) - f_u - ua_u) / g_u; //surge force
      Tz = ((lambda_r * e_r) - f_r - ua_r) / g_r; //yaw rate moment
      
      if (Tx > 73){
        Tx = 73;
      }
      if (Tx < -60){
        Tx = -60;
      }
      if (Tz > 14){
        Tz = 14;
      }
      if (Tz < -14){
        Tz = -14;
      }
      
      if (u_d == 0){
        Tx = 0;
        Tz = 0;

        Ka_u = 0;
        Ka_dot_last_u = 0;
        Ka_r = 0;
        Ka_dot_last_r = 0;
        
        e_u = 0;
        e_u_int = 0;
        e_u_last = 0;
        e_r = 0;
        e_r_int = 0;
        e_r_last = 0;
        
        sigma_u = 0;
        sigma_u_last = 0;
        sigma_r = 0;
        sigma_r_last = 0;
        
        Delta_hat_u = 0;
        Delta_hat_r = 0;
        Delta_hat_dot_last_u = 0;
        Delta_hat_dot_last_r = 0;
        
        o_dot_dot = 0;
        o_dot = 0;
        o = 0;
        o_last = 0;
        o_dot_last = 0;
        o_dot_dot_last = 0;
        
        g_dot_dot = 0;
        g_dot = 0;
        g = 0;
        g_last = 0;
        g_dot_last = 0;
        g_dot_dot_last = 0;
        
        convergence_u = 0;
        convergence_r = 0;
        eta_u = 0;
        eta_r = 0;
      }

      port_t = (Tx / 2) + (Tz / B);
      starboard_t = (Tx / (2*c)) - (Tz / (B*c));

      if (starboard_t > 36.5){
        starboard_t = 36.5;
      }
      if (starboard_t < -30){
        starboard_t = -30;
      }
      if (port_t > 36.5){
        port_t = 36.5;
      }
      if (port_t < -30){
        port_t = -30;
      }

      //Data publishing
      std_msgs::Float64 rt;
      std_msgs::Float64 lt;
      
      std_msgs::Float64 sg;
      std_msgs::Float64 hg;

      std_msgs::Float64 eu;
      std_msgs::Float64 er;

      std_msgs::Float64 su;
      std_msgs::Float64 sp;

      geometry_msgs::Pose2D ctrl_input;
      geometry_msgs::Pose2D observer;
      geometry_msgs::Pose2D observer_error;

      rt.data = starboard_t;
      lt.data = port_t;
      
      sg.data = Ka_u;
      hg.data = Ka_r;

      eu.data = e_u;
      er.data = e_r;

      su.data = sigma_u;
      sp.data = sigma_r;

      ctrl_input.x = Tx;
      ctrl_input.theta = Tz;

      observer.x = Delta_hat_u;
      observer.theta = Delta_hat_r;

      observer_error.x = Delta_tilde_u;
      observer_error.theta = Delta_tilde_r;

      right_thruster_pub.publish(rt);
      left_thruster_pub.publish(lt);

      speed_gain_pub.publish(sg);
      speed_error_pub.publish(eu);
      speed_sigma_pub.publish(su);
      heading_gain_pub.publish(hg);
      heading_error_pub.publish(er);
      heading_sigma_pub.publish(sp);
      control_input_pub.publish(ctrl_input);
      observer_pub.publish(observer);
      observer_error_pub.publish(observer_error);
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
  ros::Publisher observer_pub;
  ros::Publisher observer_error_pub;

  ros::Subscriber desired_speed_sub;
  ros::Subscriber desired_heading_sub;
  ros::Subscriber local_vel_sub;
  ros::Subscriber flag_sub;
  ros::Subscriber ardu_sub;
  ros::Subscriber start_sub;
};

// Main
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "obs_finite_time_asmc");
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

