#include <math.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>
#include "nav_msgs/Odometry.h"

int zed2Boat(){
//void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "boat";
  transformStamped.child_frame_id = "zed_camera_center";
  transformStamped.transform.translation.x = 0.52;
  transformStamped.transform.translation.y = 0;
  transformStamped.transform.translation.z = -0.22;

  tf2::Quaternion q;
  q.setRPY(3.141592, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "zed2boat_tf2_broadcaster");
  
  ros::NodeHandle node;
  while (ros::ok()){
    zed2Boat();
  }

  ros::spin();
  return 0;
}
