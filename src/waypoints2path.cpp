#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Path.h"

class Waypoint2Path{
public:
  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  Waypoint2Path(){
  path_pub = node.advertise<nav_msgs::Path>("waypoint_path", 1000);
  sub = node.subscribe("waypoint", 1000, &Waypoint2Path::odomCallback, this);
  }

  void odomCallback(const geometry_msgs::Pose2D::ConstPtr& msg){

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.pose.position.x = msg->x;
    pose.pose.position.y = -msg->y;
    pose.pose.position.z = 0;

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    path.poses.push_back(pose);
    
    path_pub.publish(path);
  }

private:
  ros::NodeHandle node;
  ros::Publisher path_pub;
  ros::Subscriber sub;

};

int main(int argc, char** argv){

  ros::init(argc, argv, "waypoints2path");
  Waypoint2Path waypoint2Path;

  while(true){
    ros::Rate(100).sleep();
    ros::spinOnce();
  }

  return 0;
}