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

        self.control_input = 0.0
        self.velocity = 0.0
        self.flag = 0
        self.arduino = 0

        rospy.Subscriber("/usv_control/controller/control_input", Pose2D, self.input_callback)
        rospy.Subscriber("/vectornav/ins_2d/local_vel", Vector3, self.vel_callback)
        rospy.Subscriber("/arduino_br/ardumotors/flag", UInt8, self.flag_callback)
        rospy.Subscriber("arduino", UInt8, self.arduino_callback)

        self.d_speed_pub = rospy.Publisher("/guidance/desired_speed", Float64, queue_size=10)


    def input_callback(self, _control_input):
        self.control_input = _control_input.x

    def vel_callback(self, _vel):
        self.velocity = _vel.x

    def flag_callback(self, _flag):
        self.flag = _flag

    def arduino_callback(self, _arduino):
        self.arduino = _arduino


    def desired(self, _speed):
        self.ds = _speed
        #self.dh = heading
        self.d_speed_pub.publish(self.ds)
        #self.d_heading_pub.publish(self.dh)

def main():
    rospy.init_node('trial_2x5', anonymous=False)
    rate = rospy.Rate(100)
    t = Test()
    dir_name = os.path.dirname(__file__)
    profile = []
    bag = rosbag.Bag(dir_name + '/mat/trial2x5_20.bag','w')
    trial_1 = rosbag.Bag(dir_name + '/mat/trial1.bag')
    for topic, msg, ti in trial_1.read_messages(topics=['e']):
        profile.append(msg.data*5)
    u = Float64()
    y = Float64()
    r = Float64()
    e = Float64()
    r_f = 0.0 #Filtered reference
    r_dot = 0.0
    r_dot_last = 0.0
    a = 20.0
    b = 20.0
    time_step = 0.01
    time.sleep(10)
    rospy.logwarn("Starting")
    if t.testing:
        start_time = rospy.Time.now().secs
        i = 0
        while (not rospy.is_shutdown()) and (i < len(profile)):
            if (t.flag != 0) and (t.arduino != 0):
                r_dot = b*profile[i] - a*r_f
                r_f = time_step*(r_dot + r_dot_last)/2 + r_f
                r_dot_last = r_dot
                u.data = t.control_input
                y.data = t.velocity
                r.data = r_f
                e.data = r.data - y.data
                bag.write('u', u)
                bag.write('y', y)
                bag.write('r', r)
                bag.write('e', e)
                t.desired(r_f)
                i = i + 1
            rate.sleep()
        bag.close()
        t.desired(0.0)
        t.testing = False
        rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
