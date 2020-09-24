#!/usr/bin/env python

import os
import time
import rospy
import rosbag
import math
import scipy.io as sio
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float64

#Variable Speed and Variable Heading
#Experiment 1 velocity profile for data driven optimization
class Test:
    def __init__(self):
        self.testing = True

        self.ds = 0.0

        self.control_input = 0.0
        self.velocity = 0.0

        rospy.Subscriber("/usv_control/controller/control_input", Pose2D, self.input_callback)
        rospy.Subscriber("/vectornav/ins_2d/local_vel", Vector3, self.vel_callback)


        self.d_speed_pub = rospy.Publisher("/guidance/desired_speed", Float64, queue_size=10)


    def input_callback(self, _control_input):
        self.control_input = _control_input.x

    def vel_callback(self, _vel):
        self.velocity = _vel.x

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
    bag = rosbag.Bag(dir_name + '/mat/trial2x5.bag','w')
    trial_1 = rosbag.Bag(dir_name + '/mat/trial1.bag')
    for topic, msg, ti in trial_1.read_messages(topics=['e']):
        profile.append(msg.data*5)
    u = Float64()
    y = Float64()
    r = Float64()
    e = Float64()
    time.sleep(2)
    if t.testing:
        start_time = rospy.Time.now().secs
        i = 0
        while (not rospy.is_shutdown()) and (i < len(profile)):
            u.data = t.control_input
            y.data = t.velocity
            r.data = profile[i]
            e.data = r.data - y.data
            bag.write('u', u)
            bag.write('y', y)
            bag.write('r', r)
            bag.write('e', e)
            t.desired(profile[i])
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