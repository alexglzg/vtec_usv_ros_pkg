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

  //Controller gains
  static const float kp_u = 1.1;
  static const float kd_u = 0.1;
  static const float ki_u = 0.2;
  static const float kp_psi = 0.8;
  static const float kd_psi = 3;
    
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
      Xu = -25;
      Xuu = 0;
      float u_abs = std::abs(u);
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
      if (std::abs(e_psi) > 3.141592){
          e_psi = (e_psi/std::abs(e_psi))*(std::abs(e_psi)-2*3.141592);
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
  }


private:
  ros::NodeHandle n;

  ros::Publisher right_thruster_pub;
  ros::Publisher left_thruster_pub;
  ros::Publisher speed_error_pub;
  ros::Publisher heading_error_pub;

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
  ros::init(argc, argv, "pid_fl");
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
