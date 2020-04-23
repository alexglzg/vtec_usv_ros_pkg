#!/usr/bin/env python

import os
import time
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64, Float32MultiArray

class Test:
    def __init__(self):
        self.testing = True

        self.path_pub = rospy.Publisher("/mission/waypoints", Float32MultiArray, queue_size=10)

    def desired(self, path):
    	self.path_pub.publish(path)

def main():
    rospy.init_node('waypoints3', anonymous=False)
    rate = rospy.Rate(20) # 100hz
    t = Test()
    path_array = Float32MultiArray()
    path_array.layout.data_offset = 9
    path_array.data = [2,2,6,-15,1,-25,6,-35, 0]
    while not rospy.is_shutdown() and t.testing:
        time.sleep(1)
        start_time = rospy.Time.now().secs
        while (rospy.Time.now().secs - start_time) <= 45 and not rospy.is_shutdown():
            t.desired(path_array)
            rate.sleep()
        t.testing = False
        rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
