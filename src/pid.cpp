#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"


class ProportionalIntegralDerivative
{
public:
  //Thruster outputs
  float Tstbd;
  float Tport;

  //Sensor feedback
  float theta;
  float u;
  float v;
  float r;

  static const float time_step = 0.01;

  int testing;
  int arduino;

  //Tracking variables
  float u_d;
  float psi_d;

  //Auxiliry variables
  float e_u_int;
  float e_u_dot;
  float e_u_last;
  float psi_d_last;

  //Model pysical parameters
  static const float B = 0.41;
  static const float c = 0.78;

  //Controller gains
  static const float kp_u = 80.0;
  static const float kd_u = 40.0;
  static const float ki_u = 50.0;
  static const float kp_psi = 45.0;
  static const float kd_psi = 45.0;
    
  float o_dot_dot;
  float o_dot;
  float o;
  float o_last;
  float o_dot_last;
  float o_dot_dot_last;
  static const float f1 = 2;
  static const float f2 = 2;
  static const float f3 = 2;

  float f_dot_dot;
  float f_dot;
  float f;
  float f_last;
  float f_dot_last;
  float f_dot_dot_last;
  static const float g1 = 2;
  static const float g2 = 2;
  static const float g3 = 2;

  float Tx;
  float Tz;
  float ua_u;
  float ua_psi;

  ProportionalIntegralDerivative()
  {
    //ROS Publishers for each required sensor data
    right_thruster_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/right_thruster", 1000);
    left_thruster_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/left_thruster", 1000);
    speed_error_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/speed_error", 1000);
    heading_error_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/heading_error", 1000);
    control_input_pub = n.advertise<geometry_msgs::Pose2D>("/usv_control/controller/control_input", 1000);

    desired_speed_sub = n.subscribe("/guidance/desired_speed", 1000, &ProportionalIntegralDerivative::desiredSpeedCallback, this);
    desired_heading_sub = n.subscribe("/guidance/desired_heading", 1000, &ProportionalIntegralDerivative::desiredHeadingCallback, this);
    ins_pose_sub = n.subscribe("/vectornav/ins_2d/ins_pose", 1000, &ProportionalIntegralDerivative::insCallback, this);
    local_vel_sub = n.subscribe("/vectornav/ins_2d/local_vel", 1000, &ProportionalIntegralDerivative::velocityCallback, this);
    flag_sub = n.subscribe("/arduino_br/ardumotors/flag", 1000, &ProportionalIntegralDerivative::flagCallback, this);
    ardu_sub = n.subscribe("arduino", 1000, &ProportionalIntegralDerivative::arduinoCallback, this);

    u_d = 0;
    psi_d = 0;
    testing = 0;
    arduino = 0;
    Tx = 0;
    Tz = 0;
    ua_u = 0;
    ua_psi = 0;
    e_u_int = 0;
    e_u_dot = 0;
    e_u_last = 0;
  }

  void desiredSpeedCallback(const std_msgs::Float64::ConstPtr& _ud)
  {
    u_d = _ud -> data;
  }

  void desiredHeadingCallback(const std_msgs::Float64::ConstPtr& _psid)
  {
    psi_d = _psid -> data;
  }

  void insCallback(const geometry_msgs::Pose2D::ConstPtr& _ins)
  {
    theta = _ins -> theta;
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

      float e_u = u_d - u;
      e_u_int = (time_step)*(e_u + e_u_last)/2 + e_u_int; //integral of the surge speed error
      e_u_dot = (e_u - e_u_last) / time_step; //derivate of the surge speed error
      e_u_last = e_u;
      
      o_dot_dot = (((e_u_dot - o_last) * f1) - (f3 * o_dot_last)) * f2;
      o_dot = (time_step)*(o_dot_dot + o_dot_dot_last)/2 + o_dot;
      o = (time_step)*(o_dot + o_dot_last)/2 + o;
      e_u_dot = o;
      o_last = o;
      o_dot_last = o_dot;
      o_dot_dot_last = o_dot_dot;

      float e_psi = psi_d - theta;
      if (std::abs(e_psi) > 3.141592){
          e_psi = (e_psi/std::abs(e_psi))*(std::abs(e_psi)-2*3.141592);
      }

      float r_d = (psi_d - psi_d_last) / time_step; //derivate of the desired heading
      psi_d_last = psi_d;

      f_dot_dot = (((r_d - f_last) * g1) - (g3 * f_dot_last)) * g2;
      f_dot = (time_step)*(f_dot_dot + f_dot_dot_last)/2 + f_dot;
      f = (time_step)*(f_dot + f_dot_last)/2 + f;
      r_d = f;
      f_last = f;
      f_dot_last = f_dot;
      f_dot_dot_last = f_dot_dot;

      float e_psi_dot = r_d - r; //derivate of the heading error

      ua_u = (kp_u * e_u) + (ki_u * e_u_int) + (kd_u * e_u_dot);
      ua_psi = (kp_psi * e_psi) + (kd_psi * e_psi_dot);
    
      Tx = ua_u; //surge force
      Tz = 0.0;//ua_psi; //yaw rate moment
      
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
        e_u_dot = 0;
        o_dot_dot = 0;
        o_dot = 0;
        o = 0;
        o_last = 0;
        o_dot_last = 0;
        o_dot_dot_last = 0;
      }

      ROS_INFO("e_dot %f, e_int %f", e_u_dot, e_u_int);


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

      geometry_msgs::Pose2D ctrl_input;

      rt.data = Tstbd;
      lt.data = Tport;
      
      eu.data = e_u;
      epsi.data = e_psi;

      ctrl_input.x = Tx;
      ctrl_input.theta = Tz;

      right_thruster_pub.publish(rt);
      left_thruster_pub.publish(lt);

      speed_error_pub.publish(eu);
      heading_error_pub.publish(epsi);
      control_input_pub.publish(ctrl_input);
    }
  }


private:
  ros::NodeHandle n;

  ros::Publisher right_thruster_pub;
  ros::Publisher left_thruster_pub;
  ros::Publisher speed_error_pub;
  ros::Publisher heading_error_pub;
  ros::Publisher control_input_pub;

  ros::Subscriber desired_speed_sub;
  ros::Subscriber desired_heading_sub;
  ros::Subscriber ins_pose_sub;
  ros::Subscriber local_vel_sub;
  ros::Subscriber flag_sub;
  ros::Subscriber ardu_sub;

};

//Main
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pid");
  ProportionalIntegralDerivative proportionalIntegralDerivative;
  int rate = 100;
  ros::Rate loop_rate(rate);

  while (ros::ok())
  {
    proportionalIntegralDerivative.control();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
