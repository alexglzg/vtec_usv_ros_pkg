#include <math.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "gazebo_msgs/ModelState.h"

class States{
public:
  gazebo_msgs::ModelState states;
  States(){
  gazebo_pub = node.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
  sub = node.subscribe("/vectornav/ins_2d/NED_pose", 1000, &States::odomCallback, this);
  }

  void odomCallback(const geometry_msgs::Pose2D::ConstPtr& msg){
    
    tf2::Quaternion q;
    q.setRPY(0, 0, -msg->theta);

    states.model_name = "vtec_s3";

    states.pose.position.x = msg->x;
    states.pose.position.y = -msg->y;
    states.pose.position.z = 0;

    states.pose.orientation.x = q.x();
    states.pose.orientation.y = q.y();
    states.pose.orientation.z = q.z();
    states.pose.orientation.w = q.w();

    gazebo_pub.publish(states);
  }

private:
  ros::NodeHandle node;
  ros::Publisher gazebo_pub;
  ros::Subscriber sub;

};

int main(int argc, char** argv){

  ros::init(argc, argv, "boat_gazebo_broadcaster");
  States states_;

  while (ros::ok()){
    ros::Rate(100).sleep();
    ros::spinOnce();
  }

  return 0;
}