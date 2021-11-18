#!/usr/bin/env python

import os
import time
import rospy
import rosbag
import math
import scipy.io as sio
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float64, UInt8

#Variable Speed and Variable Heading
#Experiment 1 velocity profile for data driven optimization
class Test:
    def __init__(self):
        self.testing = True

        self.ds = 0.0
        self.dh = 0.0

        self.x_pos = 0.0
        self.y_pos = 0.0

        self.control_input_speed = 0.0
        self.velocity = 0.0
        self.control_input_heading = 0.0
        self.yaw_rate = 0.0
        self.heading = 0.0
        self.flag = 0
        self.arduino = 0

        rospy.Subscriber("/usv_control/controller/control_input", Pose2D, self.input_callback)
        rospy.Subscriber("/vectornav/ins_2d/local_vel", Vector3, self.vel_callback)
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.ned_callback)
        rospy.Subscriber("/arduino_br/ardumotors/flag", UInt8, self.flag_callback)
        rospy.Subscriber("arduino", UInt8, self.arduino_callback)

        self.d_speed_pub = rospy.Publisher("/guidance/desired_speed", Float64, queue_size=10)
        self.d_heading_pub = rospy.Publisher("/guidance/desired_heading", Float64, queue_size=10)

    def input_callback(self, _control_input):
        self.control_input_speed = _control_input.x
        self.control_input_heading = _control_input.theta

    def vel_callback(self, _vel):
        self.velocity = _vel.x
        self.yaw_rate = _vel.z

    def ned_callback(self, _ned):
        self.x_pos = _ned.x
        self.y_pos = _ned.y
        self.heading = _ned.theta
    
    def flag_callback(self, _flag):
        self.flag = _flag

    def arduino_callback(self, _arduino):
        self.arduino = _arduino

    def desired(self, _speed, _heading):
        self.ds = _speed
        self.dh = _heading
        self.d_speed_pub.publish(self.ds)
        self.d_heading_pub.publish(self.dh)

def main():
    rospy.init_node('trialnu_1', anonymous=False)
    rate = rospy.Rate(100)
    t = Test()
    dir_name = os.path.dirname(__file__)
    profile = sio.loadmat(dir_name + '/mat/profile.mat')
    profile = profile['profile']
    bag = rosbag.Bag(dir_name + '/mat/trialnu_1.bag','w')
    u_speed = Float64()
    y_speed = Float64()
    r_speed = Float64()
    e_speed = Float64()
    u_heading = Float64()
    y_heading = Float64()
    r_heading = Float64()
    e_heading = Float64()
    p_vector = Pose2D()

    time.sleep(10)
    rospy.logwarn("Starting")
    if t.testing:
        start_time = rospy.Time.now().secs
        i = 0
        while (not rospy.is_shutdown()) and (i < len(profile)):
            if (t.flag != 0) and (t.arduino != 0):
                if (profile[i] > 0.01):
                    u_speed.data = t.control_input_speed
                    y_speed.data = t.velocity
                    r_speed.data = profile[i]
                    e_speed.data = r_speed.data - y_speed.data
                    u_heading.data = t.control_input_heading
                    y_heading.data = t.yaw_rate
                    r_heading.data = -profile[i]*0.5
                    e_heading.data = r_heading.data - y_heading.data
                    p_vector.x = t.x_pos
                    p_vector.y = t.y_pos
                    p_vector.theta = t.heading
                    bag.write('u_speed', u_speed)
                    bag.write('y_speed', y_speed)
                    bag.write('r_speed', r_speed)
                    bag.write('e_speed', e_speed)
                    bag.write('u_heading', u_heading)
                    bag.write('y_heading', y_heading)
                    bag.write('r_heading', r_heading)
                    bag.write('e_heading', e_heading)
                    bag.write('position', p_vector)
                    t.desired(profile[i],-profile[i]*0.5)
                else:
                    t.desired(0.0,0.0)
                    if i > len(profile)/2:
                        bag.close()
                        rospy.logwarn("Finished")
                        i = len(profile)
                        t.testing = False
                i = i + 1
            rate.sleep()
        t.desired(0.0,0.0)
        t.testing = False
        rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
