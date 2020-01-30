#!/usr/bin/env python

import os
import time
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64
import math

#Constant Speed and Constant angular
#0.7 m/s and pi/8 rad/s
class Test:
    def __init__(self):
        self.testing = True

        self.ds = 0
        self.dh = 0

        self.reference_angular = 0

        #rospy.Subscriber("ref", Pose2D, self.ref_callback)

        self.d_speed_pub = rospy.Publisher("desired_speed", Float64, queue_size=10)
        self.d_angular_pub = rospy.Publisher("desired_angular", Float64, queue_size=10)

    #def ref_callback(self, refh):
        #self.reference_angular = refh.theta

    def desired(self, speed, angular):
        #while not rospy.is_shutdown():
        self.ds = speed
        self.dh = angular
        self.d_speed_pub.publish(self.ds)
        self.d_angular_pub.publish(self.dh)

def main():
    rospy.init_node('cu_cr', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    t = Test()
    while not rospy.is_shutdown() and t.testing:
        start_time = rospy.Time.now().secs
        while (rospy.Time.now().secs - start_time) <= 2 and not rospy.is_shutdown():
            t.desired(0,0)
            rate.sleep()
        while (rospy.Time.now().secs - start_time) <= 10 and not rospy.is_shutdown():
            t.desired(0.7,math.pi/8)
            rate.sleep()
        t.desired(0,0)
        t.testing = False
        rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
