#include <iostream>
#include "vn/sensors.h"
#include "vn/thread.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include <math.h>
#include <Eigen/Dense>

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;
using namespace Eigen;

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "imu");

  	ros::NodeHandle n;
	
	const string SensorPort = "/dev/ttyUSB0";                  // Linux format for virtual (USB) serial port.
	const uint32_t SensorBaudrate = 115200;

	VnSensor vs;
	vs.connect(SensorPort, SensorBaudrate);

	//ROS Publishers for each required sensor data
	ros::Publisher ins_pos_pub = n.advertise<geometry_msgs::Pose2D>("ins_pose", 1000);
	ros::Publisher ins_vel_pub = n.advertise<geometry_msgs::Vector3>("ins_vel", 1000);
	ros::Publisher ins_acc_pub = n.advertise<geometry_msgs::Vector3>("ins_acc", 1000);
	ros::Publisher ins_ar_pub = n.advertise<geometry_msgs::Vector3>("ins_ar", 1000);
	ros::Publisher local_vel_pub = n.advertise<geometry_msgs::Vector3>("local_vel", 1000);
	ros::Publisher NED_pose_pub = n.advertise<geometry_msgs::Pose2D>("NED_pose", 1000);
	ros::Publisher ECEF_pose_pub = n.advertise<geometry_msgs::Pose2D>("ECEF_pose", 1000);
	ros::Publisher ref_pub = n.advertise<geometry_msgs::Pose2D>("ref", 1000);


	//Transformation of coordinates Geodetic-Ecef-NED for the reference
	int R_Ea = 6378137; //Earth radious
	float eccentricity = 0.08181919; //The eccentricity of the geodetic plane
	InsStateLlaRegister ref; //reference (starting) data
	//TODO check InsState vs InsState
	ref = vs.readInsStateLla();

	InsStateEcefRegister Ecefref;
	Ecefref = vs.readInsStateEcef();
	
	float Ecefrefx = 0;
	float Ecefrefy = 0;
	float Ecefrefz = 0;
	float refposx = 0;
	float refposy = 0;

	for (int i=1;i<=20;i++)
	{
		Ecefrefx = Ecefrefx + Ecefref.position.x;
		Ecefrefy = Ecefrefy + Ecefref.position.y;
		Ecefrefz = Ecefrefz + Ecefref.position.z;
		refposx = refposx + ref.position.x;
		refposy = refposy + ref.position.y;
	}

	Ecefrefx = Ecefrefx/20;
	Ecefrefy = Ecefrefy/20;
	Ecefrefz = Ecefrefz/20;
	refposx = refposx/20;
	refposy = refposy/20;

	float refx = (3.141592 / 180)*(refposx);
	float refy = (3.141592 / 180)*(refposy);

	Vector3f Pe_ref;
	Pe_ref << Ecefrefx,
			  Ecefrefy,
			  Ecefrefz;

	Matrix3f Rne;
	Rne << -sin(refx) * cos(refy), -sin(refx) * sin(refy), cos(refx),
		   -sin(refy), cos(refy), 0,
		   -cos(refx) * cos(refy), -cos(refx) * sin(refy), -sin(refx);

	geometry_msgs::Pose2D ins_ref;
	ins_ref.x = refposx;
	ins_ref.y = refposy;
	ins_ref.theta = (3.141592 / 180)*(ref.yawPitchRoll.x);

	ros::Rate loop_rate(250);

  while (ros::ok())
  {

	InsStateLlaRegister ins; //Inertial Navigation System (INS) variable declaration
	InsStateEcefRegister Ecef; //INS with Ecef coordinates

	ins = vs.readInsStateLla(); //Variable that reads the INS data
	Ecef = vs.readInsStateEcef();

	geometry_msgs::Pose2D ins_pose; //inertial navigation system pose (latitude, longitude, yaw)
	geometry_msgs::Vector3 ins_vel; //velocity/speed in a global reference frame
	geometry_msgs::Vector3 ins_acc; //acceleration
	geometry_msgs::Vector3 ins_ar; //angular rate
	geometry_msgs::Vector3 local_vel; //veocity/speed in a local reference frame
	geometry_msgs::Pose2D NED_pose; //pose in a local reference frame (x, y, yaw)
	geometry_msgs::Pose2D ECEF_pose;

	ins_pose.x = ins.position.x; //latitude
	ins_pose.y = ins.position.y; //longitude
	ins_pose.theta = (3.141592 / 180)*(ins.yawPitchRoll.x); //yaw converted from degrees into radians

	float rad_pose_x = (3.141592 / 180)*(ins.position.x); //latitude in radians
	float rad_pose_y = (3.141592 / 180)*(ins.position.y); //longitude in radians

	ins_vel.x = ins.velocity.x; //velocity in North direction
	ins_vel.y = ins.velocity.y; //velocity in East direction
	ins_vel.z = ins.velocity.z; //velocity in Down direction

	ins_acc.x = ins.accel.x; //acceleration in the x axis
	ins_acc.y = ins.accel.y; //acceleration in the y axis
	ins_acc.z = ins.accel.z; //acceleration in the z axis

	ins_ar.x = ins.angularRate.x; //roll rate
	ins_ar.y = ins.angularRate.y; //pitch rate
	ins_ar.z = ins.angularRate.z; //yaw rate (r)

//local velocity and yaw rate
	Vector3f eta_dot; //vector declaration of eta' = [x' y' psi'] (3 DOF global reference frame)
	eta_dot << ins_vel.x,
			  ins_vel.y,
			  ins_ar.z;
	Matrix3f J; //matrix of transformation between reference frames
	J << cos(ins_pose.theta), -sin(ins_pose.theta), 0,
		 sin(ins_pose.theta), cos(ins_pose.theta), 0,
		 0, 0, 1;
	Vector3f upsilon; //vector upsilon = [u v r] (3 DOF local reference frame)
	upsilon = J.inverse()*eta_dot; //transformation into local reference frame
	float u = upsilon(0); //surge velocity
	float v = upsilon(1); //sway velocity
	float r = upsilon(2); //yaw rate
	local_vel.x = u;
	local_vel.y = v;
	local_vel.z = r;

//Ecef coordinates
	Vector3f Pe;
	Pe <<   Ecef.position.x,
			Ecef.position.y,
			Ecef.position.z;

	ECEF_pose.x = Pe(0);
	ECEF_pose.y = Pe(1);
	ECEF_pose.theta = ins_pose.theta;

//NED from Ecef coordinates
	Vector3f NED;
	NED = Rne * (Pe - Pe_ref);
	float N = NED(0);
	float E = NED(1);
	NED_pose.x = N;
	NED_pose.y = E;
	NED_pose.theta = ins_pose.theta;

//Data publishing
    ins_pos_pub.publish(ins_pose);
    //ins_vel_pub.publish(ins_vel);
    //ins_acc_pub.publish(ins_acc);
    //ins_ar_pub.publish(ins_ar);
    local_vel_pub.publish(local_vel);
    NED_pose_pub.publish(NED_pose);
    //ECEF_pose_pub.publish(ECEF_pose);
    ref_pub.publish(ins_ref);

    ros::spinOnce();

    loop_rate.sleep();
  }

	vs.disconnect();

	return 0;
}
