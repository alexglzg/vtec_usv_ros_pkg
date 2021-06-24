#!/usr/bin/env python

import os
import time
import rospy
import numpy as np
from std_msgs.msg import Float64, UInt8

#Constant Speed and Constant Heading

desired_u = rospy.get_param("cu_cr/desired_u", 0.7)
desired_r = rospy.get_param("cu_cr/desired_r", 8)

class Test:
    def __init__(self):
        self.testing = True

        self.ds = 0
        self.dh = 0

        self.reference_heading = 0

        self.d_speed_pub = rospy.Publisher("/guidance/desired_speed", Float64, queue_size=10)
        self.d_heading_pub = rospy.Publisher("/guidance/desired_heading", Float64, queue_size=10)
        self.start_pub = rospy.Publisher("/start_prescribed", UInt8, queue_size=10)

    def desired(self, speed, heading, start):
        #while not rospy.is_shutdown():
        self.ds = speed
        self.dh = heading
        self.d_speed_pub.publish(self.ds)
        self.d_heading_pub.publish(self.dh)
        self.start_pub.publish(start)

def main():
    rospy.init_node('cu_cr', anonymous=False)
    rate = rospy.Rate(100) # 100hz
    t = Test()
    time.sleep(8)
    flag = 0
    while not rospy.is_shutdown() and t.testing:
        start_time = rospy.Time.now().secs
        while (rospy.Time.now().secs - start_time) <= 10 and not rospy.is_shutdown():
            if flag < 2:
                t.desired(desired_u,np.pi/desired_r, 1)
                flag = flag + 1
            else:
                t.desired(desired_u,np.pi/desired_r, 2)
            rate.sleep()
        t.desired(0,0,0)
        t.testing = False
        rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
