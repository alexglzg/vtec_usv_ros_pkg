#!/usr/bin/env python

import os
import time
import rospy
import numpy as np
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64, Float32MultiArray

class Test:
    def __init__(self):
        self.testing = True

        self.path_pub = rospy.Publisher("/mission/waypoints", Float32MultiArray, queue_size=10)
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.ned_callback)

    def ned_callback(self, gps):
        self.ned_x = gps.x
        self.ned_y = gps.y
        self.yaw = gps.theta

    def desired(self, path):
    	self.path_pub.publish(path)

def main():
    rospy.init_node('curved_waypoints', anonymous=False)
    rate = rospy.Rate(100) # 100hz
    t = Test()
    path = []
    r = 4
    d = r*2
    step = .01
    x_0 = 3
    y_0 = 1
    offset1 = x_0 + r
    offset2 = x_0 + r + d
    x = 0.0
    i = 0
    path_array = Float32MultiArray()
    time.sleep(2)
    while not rospy.is_shutdown() and t.testing:
        if i == 0:
            while x < d:
                y = np.sqrt(r**2 - np.power(x+x_0-offset1,2))
                path.append(x+x_0)
                path.append(y+y_0)
                x = x + step
            d = d + 2*r
            while x < d-step:
                y = -np.sqrt(r**2 - np.power(x+x_0-offset2,2))
                path.append(x+x_0)
                path.append(y+y_0)
                x = x + step
            path.append(0)
            path_array.layout.data_offset = len(path)
            path_array.data = path
            t.desired(path_array)
            d = d - 2*r
            last = len(path) - 1
            i = 1

        elif i == 1:
            dx = t.ned_x - (x_0+4*r)
            dy = t.ned_y - (y_0)
            distance = np.sqrt(dx**2 + dy**2)
            if distance < 1:
                i = 2
                path = []

        elif i == 2:
            while x > d:
                y = np.sqrt(r**2 - np.power(x+x_0-offset2,2))
                path.append(x+x_0)
                path.append(y+y_0)
                x = x - step
            d = d - 2*r
            while x > d+step:
                y = -np.sqrt(r**2 - np.power(x+x_0-offset1,2))
                path.append(x+x_0)
                path.append(y+y_0)
                x = x - step
            path.append(x_0)
            path.append(y_0)
            path.append(0)
            path_array.layout.data_offset = len(path)
            path_array.data = path
            t.desired(path_array)
            i = 3
            t.testing = False
        rate.sleep()
        #rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
