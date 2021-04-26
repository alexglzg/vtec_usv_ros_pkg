#!/usr/bin/env python

import os
import time
import rospy
import numpy as np
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64

#Constant Speed and Constant Heading
#1 m/s and pi/8 rads
class Test:
    def __init__(self):
        self.testing = True

        self.ds = 0
        self.dh = 0

        self.reference_heading = 0

        self.d_speed_pub = rospy.Publisher("/guidance/desired_speed", Float64, queue_size=10)
        self.d_heading_pub = rospy.Publisher("/guidance/desired_heading", Float64, queue_size=10)

    def desired(self, speed, heading):
        #while not rospy.is_shutdown():
        self.ds = speed
        self.dh = heading
        self.d_speed_pub.publish(self.ds)
        self.d_heading_pub.publish(self.dh)

def main():
    rospy.init_node('cu_cr', anonymous=False)
    rate = rospy.Rate(100) # 100hz
    t = Test()
    while not rospy.is_shutdown() and t.testing:
        start_time = rospy.Time.now().secs
        while (rospy.Time.now().secs - start_time) <= 2 and not rospy.is_shutdown():
            t.desired(0,0)
            rate.sleep()
        while (rospy.Time.now().secs - start_time) <= 10 and not rospy.is_shutdown():
            t.desired(1,np.pi/8)
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
