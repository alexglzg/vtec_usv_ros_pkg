#!/usr/bin/env python

import os
import time
import rospy
import math
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64

#Constant Speed and Variable Heading
#0.7 m/s and imu +- 0.2 rads
class Test:
    def __init__(self):
        self.testing = True

        self.ds = 0
        self.dh = 0

        self.reference_heading = 0

        rospy.Subscriber("ref", Pose2D, self.ref_callback)

        self.d_speed_pub = rospy.Publisher("desired_speed", Float64, queue_size=10)
        self.d_heading_pub = rospy.Publisher("desired_heading", Float64, queue_size=10)

    def ref_callback(self, refh):
        self.reference_heading = refh.theta

    def desired(self, speed, heading):
        self.ds = speed
        self.dh = heading
        self.d_speed_pub.publish(self.ds)
        self.d_heading_pub.publish(self.dh)

def main():
    rospy.init_node('cs_vh_3', anonymous=True)
    t = Test()
    offset = 0
    sinewave = 0
    if t.testing:
        start_time = rospy.Time.now().secs
        while (rospy.Time.now().secs - start_time) <= 1:
            t.desired(0,t.reference_heading)
            time.sleep(0.1)
        while (rospy.Time.now().secs - start_time) <= 5:
            t.desired(0.7,t.reference_heading)
            time.sleep(0.1)
        while (rospy.Time.now().secs - start_time) <= 45:
            offset += math.pi/250
            sinewave = 0.2*math.sin(offset)
            t.desired(0.7,t.reference_heading+sinewave)
            time.sleep(0.1)
        t.desired(0,t.reference_heading+sinewave)
        t.testing = False
        rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
