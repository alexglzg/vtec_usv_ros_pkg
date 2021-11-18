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
#Experiment 2 erros as reference for data driven optimization
class Test:
    def __init__(self):
        self.testing = True

        self.ds = 0.0
        self.dh = 0.0

        self.x = 0.0
        self.y = 0.0

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
        self.x = _ned.x
        self.y = _ned.y
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
    rospy.init_node('trialnu_2', anonymous=False)
    rate = rospy.Rate(100)
    t = Test()
    dir_name = os.path.dirname(__file__)
    profile_speed = []
    profile_heading = []
    bag = rosbag.Bag(dir_name + '/mat/trialnu_2_25.bag','w')
    trial_1 = rosbag.Bag(dir_name + '/mat/trialnu_1.bag')
    for topic, msg, ti in trial_1.read_messages(topics=['e_speed']):
        profile_speed.append(msg.data)
    for topic, msg, ti in trial_1.read_messages(topics=['e_heading']):
        profile_heading.append(msg.data)
    u_speed = Float64()
    y_speed = Float64()
    r_speed = Float64()
    e_speed = Float64()
    u_heading = Float64()
    y_heading = Float64()
    r_heading = Float64()
    e_heading = Float64()
    pos = Pose2D()

    rs_f = 0.0 #Filtered reference
    rs_dot = 0.0
    rs_dot_last = 0.0
    rh_f = 0.0 #Filtered reference
    rh_dot = 0.0
    rh_dot_last = 0.0
    a = 2.5
    b = 2.5
    time_step = 0.01
    time.sleep(10)
    rospy.logwarn("Starting")
    if t.testing:
        start_time = rospy.Time.now().secs
        i = 0
        while (not rospy.is_shutdown()) and (i < len(profile_speed)):
            if (t.flag != 0) and (t.arduino != 0):
                rs_dot = b*profile_speed[i] - a*rs_f
                rs_f = time_step*(rs_dot + rs_dot_last)/2 + rs_f
                rs_dot_last = rs_dot
                rh_dot = b*profile_heading[i] - a*rh_f
                rh_f = time_step*(rh_dot + rh_dot_last)/2 + rh_f
                rh_dot_last = rh_dot
                u_speed.data = t.control_input_speed
                y_speed.data = t.velocity
                r_speed.data = rs_f
                e_speed.data = r_speed.data - y_speed.data
                u_heading.data = t.control_input_heading
                y_heading.data = t.yaw_rate
                r_heading.data = rh_f
                e_heading.data = r_heading.data - y_heading.data
                pos.x = t.x
                pos.y = t.y
                pos.theta = t.heading
                bag.write('u_speed', u_speed)
                bag.write('y_speed', y_speed)
                bag.write('r_speed', r_speed)
                bag.write('e_speed', e_speed)
                bag.write('u_heading', u_heading)
                bag.write('y_heading', y_heading)
                bag.write('r_heading', r_heading)
                bag.write('e_heading', e_heading)
                bag.write('position', pos)

                t.desired(rs_f,rh_f)
                i = i + 1
            rate.sleep()
        bag.close()
        t.desired(0.0,0.0)
        t.testing = False
        rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
