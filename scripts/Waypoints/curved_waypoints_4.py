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
        self.des_altitude_pub = rospy.Publisher("/ad", Float64, queue_size=10)
        rospy.Subscriber("/vectornav/ins_2d/NED_pose", Pose2D, self.ned_callback)

    def ned_callback(self, gps):
        self.ned_x = gps.x
        self.ned_y = gps.y
        self.yaw = gps.theta

    def desired(self, path):
    	self.path_pub.publish(path)

def main():
    rospy.init_node('curved_waypoints_4', anonymous=False)
    rate = rospy.Rate(100) # 100hz
    t = Test()
    path = []
    r = 4
    d = r*2
    step = np.pi/200
    x_0 = 4
    y_0 = 0
    x_offset = x_0
    y_offset = d + y_0
    th = np.pi
    i = 0
    path_array = Float32MultiArray()
    time.sleep(2)
    while not rospy.is_shutdown() and t.testing:
        if i == 0:
            while th < 2*np.pi:
                s = (d*2)/(3 - np.cos(2*th))
                x = (np.sin(2*th)/2)*s + x_offset
                y = np.cos(th)*s + y_offset
                path.append(x)
                path.append(y)
                th = th + step
            path.append(0)
            path_array.layout.data_offset = len(path)
            path_array.data = path
            t.desired(path_array)
            t.des_altitude_pub.publish(0.000000568)
            last = len(path) - 1
            i = 1

        elif i == 1:
            dx = t.ned_x - (x_0)
            dy = t.ned_y - (y_0 + 2*d)
            distance = np.sqrt(dx**2 + dy**2)
            if distance < 0.5:
                i = 2
                path = []
                th = 0

        elif i == 2:
            while th < np.pi:
                s = (d*2)/(3 - np.cos(2*th))
                x = (np.sin(2*th)/2)*s + x_offset
                y = np.cos(th)*s + y_offset
                path.append(x)
                path.append(y)
                th = th + step
            path.append(y_0)
            path.append(x_0)
            path.append(0)
            path_array.layout.data_offset = len(path)
            path_array.data = path
            t.desired(path_array)
            t.des_altitude_pub.publish(0.0000023)
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
