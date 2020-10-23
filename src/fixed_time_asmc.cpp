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

  //Tracking variables
  float u_d;
  float r_d;

  //Auxiliry variables
  float e_u_int;
  float e_u_last;
  float e_r_int;
  float e_r_last;
  static const float sqr2 = pow(2,0.5);

  //Model pysical parameters
  float Xu;
  float Nr;
  static const float X_u_dot = -2.25;
  static const float Y_v_dot = -23.13;
  static const float N_r_dot = -2.79;
  float Xuu;
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

  //Fixed-time variables
  float tc;
  float L1;
  float L1_dot;
  float epsilon;

  //Observer
  float Delta_hat_u;
  float Delta_hat_r;
  float Delta_tilde_u;
  float Delta_tilde_r;

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
    
    //ROS Subscribers
    desired_speed_sub = n.subscribe("/guidance/desired_speed", 1000, &AdaptiveSlidingModeControl::desiredSpeedCallback, this);
    desired_heading_sub = n.subscribe("/guidance/desired_heading", 1000, &AdaptiveSlidingModeControl::desiredHeadingCallback, this);
    local_vel_sub = n.subscribe("/vectornav/ins_2d/local_vel", 1000, &AdaptiveSlidingModeControl::velocityCallback, this);
    flag_sub = n.subscribe("/arduino_br/ardumotors/flag", 1000, &AdaptiveSlidingModeControl::flagCallback, this);
    ardu_sub = n.subscribe("arduino", 1000, &AdaptiveSlidingModeControl::arduinoCallback, this);

    static const float dtc = 3;
    static const float dL1 = 0.4;
    static const float dL1_dot = 0;
    static const float depsilon = 0.003;
    static const float dK2_u = 0.1;
    static const float dK2_r = 0.1;
    static const float dmu_u = 0.05;
    static const float dmu_r = 0.05;
    static const float dlambda_u = 0.01;
    static const float dlambda_r = 0.01;
    static const float deta_u = 0.4714;
    static const float deta_r = 0.1851;

    n.param("/asmc/tc", tc, dtc);
    n.param("/asmc/L1", L1, dL1);
    n.param("/asmc/L1_dot", L1_dot, dL1_dot);
    n.param("/asmc/epsilon", epsilon, depsilon);
    n.param("/asmc/K2_u", K2_u, dK2_u);
    n.param("/asmc/K2_r", K2_r, dK2_r);
    n.param("/asmc/mu_u", mu_u, dmu_u);
    n.param("/asmc/mu_r", mu_r, dmu_r);
    n.param("/asmc/lambda_u", lambda_u, dlambda_u);
    n.param("/asmc/lambda_r", lambda_r, dlambda_r);
    n.param("/asmc/eta_u", eta_u, deta_u);
    n.param("/asmc/eta_r", eta_r, deta_r);

    u_d = 0;
    r_d = 0;
    testing = 0;
    arduino = 0;

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

  void control()
  {
    if (testing == 1 && arduino == 1){
      Xu = -25;
      Xuu = 0;
      float u_abs = std::abs(u);
      if (u_abs > 1.2){
        Xu = 64.55;
        Xuu = -70.92;
      }

      Nr = (-0.52)*pow(pow(u,2) + pow(v,2),0.5);

      float g_u = (1 / (m - X_u_dot));
      float g_r = (1 / (Iz - N_r_dot));

      float f_u = (((m - Y_v_dot)*v*r + (Xuu*u_abs*u + Xu*u)) / (m - X_u_dot));
      float f_r = (((-X_u_dot + Y_v_dot)*u*v + (Nr*r)) / (Iz - N_r_dot));

      float e_u = u_d - u;
      float e_r = r_d - r;

      e_u_int = (integral_step)*(e_u + e_u_last)/2 + e_u_int; //integral of the surge speed error
      e_u_last = e_u;

      e_r_int = (integral_step)*(e_r + e_r_last)/2 + e_r_int; //integral of the heading error
      e_r_last = e_r;

      float sigma_u = e_u + lambda_u * e_u_int;
      float sigma_r = e_r + lambda_r * e_r_int;
      
      float sigma_u_abs = std::abs(sigma_u);
      float sigma_r_abs = std::abs(sigma_r);
      
      int sign_u_sm = 0;
      int sign_r_sm = 0;

      int sign_u = 0;
      int sign_r = 0;

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



      Ka_u = (1/pow(sigma_u_abs,0.5)) * (Delta_hat_u*sign_u + (eta_u/sqr2) - K2_u*sigma_u_abs);
      Ka_r = (1/pow(sigma_r_abs,0.5)) * (Delta_hat_r*sign_r + (eta_r/sqr2) - K2_r*sigma_r_abs);

      ua_u = ((-Ka_u) * pow(sigma_u_abs,0.5) * sign_u) - (K2_u*sigma_u);
      ua_r = ((-Ka_r) * pow(sigma_r_abs,0.5) * sign_r) - (K2_r*sigma_r);

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
      
      if (u_d == 0){
        Tx = 0;
        Tz = 0;
        Ka_u = 0;
        Ka_dot_last_u = 0;
        Ka_r = 0;
        Ka_dot_last_r = 0;
        e_u_int = 0;
        e_u_last = 0;
        e_r_int = 0;
        e_r_last = 0;
        Delta_hat_u = 0;
        Delta_hat_r = 0;
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
      std_msgs::Float64 rt;
      std_msgs::Float64 lt;
      
      std_msgs::Float64 sg;
      std_msgs::Float64 hg;

      std_msgs::Float64 eu;
      std_msgs::Float64 er;

      std_msgs::Float64 su;
      std_msgs::Float64 sp;

      geometry_msgs::Pose2D ctrl_input;

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

      right_thruster_pub.publish(rt);
      left_thruster_pub.publish(lt);

      speed_gain_pub.publish(sg);
      speed_error_pub.publish(eu);
      speed_sigma_pub.publish(su);
      heading_gain_pub.publish(hg);
      heading_error_pub.publish(er);
      heading_sigma_pub.publish(sp);
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

  ros::Subscriber desired_speed_sub;
  ros::Subscriber desired_heading_sub;
  ros::Subscriber local_vel_sub;
  ros::Subscriber flag_sub;
  ros::Subscriber ardu_sub;
};

// Main
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "fixed_time_asmc");
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