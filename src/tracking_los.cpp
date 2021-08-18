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

class TrackingLos
{
public:
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
  float gamma_p;

  //Auxiliry variables
  float x_e;
  float y_e;
  float beta;
  float U_d;
  float U_pseudo;
  float u_d;
  float psi_d;

  //Controller gains
  float k_U;
  float Delta_L;

  Vector2f err;
  Matrix2f err_aux_1;
  Vector2f err_aux_2;

  std_msgs::Float64 u_desired;
  std_msgs::Float64 psi_desired;

  std_msgs::Float64 x_error;
  std_msgs::Float64 y_error;

  TrackingLos()
  {
    //ROS Publishers for each required sensor data
    desired_speed_pub = n.advertise<std_msgs::Float64>("/guidance/desired_speed", 1000);
    desired_heading_pub = n.advertise<std_msgs::Float64>("/guidance/desired_heading", 1000);
    x_error_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/x_error", 1000);
    y_error_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/y_error", 1000);

    //ROS Subscribers
    desired_trajectory_sub = n.subscribe("/mission/trajectory", 1000, &TrackingLos::desiredTrajCallback, this);
    desired_trajectorydot_sub = n.subscribe("/mission/trajectory_derivative", 1000, &TrackingLos::desiredTrajDotCallback, this);
    ins_pose_sub = n.subscribe("/vectornav/ins_2d/NED_pose", 1000, &TrackingLos::insCallback, this);
    local_vel_sub = n.subscribe("/vectornav/ins_2d/local_vel", 1000, &TrackingLos::velocityCallback, this);
    flag_sub = n.subscribe("/arduino_br/ardumotors/flag", 1000, &TrackingLos::flagCallback, this);
    ardu_sub = n.subscribe("arduino", 1000, &TrackingLos::arduinoCallback, this);

    static const float dk_U = 0.1;
    static const float dDelta_L = 1.0;

    n.param("/tracking_los/k_U", k_U, dk_U);
    n.param("/tracking_los/Delta_L", Delta_L, dDelta_L);

    x_d = 0;
    y_d = 0;
    gamma_p = 0;
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
    gamma_p = std::atan2(ydot_d, xdot_d);
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

      err_aux_1 << cos(gamma_p), sin(gamma_p),
                  -sin(gamma_p), cos(gamma_p);

      err_aux_2 << x - x_d,
                  y - y_d;

      err << err_aux_1*err_aux_2;

      x_e = err(0);
      y_e = err(1);

      U_d = pow(xdot_d*xdot_d + ydot_d*ydot_d, 0.5);
      U_pseudo = (U_d - k_U*x_e)*pow(y_e*y_e + Delta_L*Delta_L, 0.5)/Delta_L;
      u_d = pow(U_pseudo*U_pseudo - v*v, 0.5);

      if (u == 0.0){
        u = 0.001;
      }
      beta = std::atan2(v,u);
      psi_d = gamma_p + std::atan(-y_e/Delta_L) - beta;
      
      if (u_d > 1.0){
        u_d = 1.0;
      }

      if (std::abs(psi_d) > 3.141592){
          psi_d = (psi_d/std::abs(psi_d))*(std::abs(psi_d) - 2*3.141592);
      }
      
      if (starting == 0){
        u_d = 0;
        psi_d = 0;
        err(0) = 0;
        err(1) = 0;
      }

      //Data publishing
      u_desired.data = u_d;
      psi_desired.data = psi_d;
      x_error.data = x_e;
      y_error.data = y_e;

      desired_speed_pub.publish(u_desired);
      desired_heading_pub.publish(psi_desired);

      x_error_pub.publish(x_error);
      y_error_pub.publish(y_error);
    }
  }

private:
  ros::NodeHandle n;

  ros::Publisher x_error_pub;
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
  ros::init(argc, argv, "tracking_los");
  TrackingLos trackingLos;
  int rate = 100;
  ros::Rate loop_rate(rate);

  while (ros::ok())
  {
    trackingLos.control();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}