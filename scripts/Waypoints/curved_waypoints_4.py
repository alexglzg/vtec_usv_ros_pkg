#!/usr/bin/env python

import os
import time
import rospy
import numpy as np
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float64, Float32MultiArray

class Test:
    def __init__(self):
        self.testing = True

        self.path_pub = rospy.Publisher("/mission/waypoints", Float32MultiArray, queue_size=10)
        self.des_altitude_pub = rospy.Publisher("/ad", Float64, queue_size=10)
        self.usv_disturbance_pub = rospy.Publisher("/usv_disturbance", Pose2D, queue_size=10)
        self.uav_disturbance_pub = rospy.Publisher("/uav_disturbance", Vector3, queue_size=10)
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
    usv_disturbance = Pose2D()
    uav_disturbance = Vector3()
    o_dot_dot = 0
    o_dot = 0
    o = 0.000000568
    o_last = 0.000000568
    o_dot_last = 0
    o_dot_dot_last = 0
    f1 = 2
    f2 = 2
    f3 = 2
    start_time = rospy.Time.now().secs
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
            a_d = 0.0000023
            o_dot_dot = (((a_d - o_last) * f1) - (f3 * o_dot_last)) * f2
            o_dot = (0.01)*(o_dot_dot + o_dot_dot_last)/2 + o_dot
            o = (0.01)*(o_dot + o_dot_last)/2 + o
            o_last = o
            o_dot_last = o_dot
            o_dot_dot_last = o_dot_dot
            t.desired(path_array)
            t.des_altitude_pub.publish(o)
            i = 3

        now = rospy.Time.now().secs - start_time
        if ((now >= 10) and (now <= 20)):
            usv_disturbance.x = -5.0
            usv_disturbance.y = 0.0
            usv_disturbance.theta = 0.0
            t.usv_disturbance_pub.publish(usv_disturbance)
            uav_disturbance.x = -0.1
            uav_disturbance.y = 0.0
            uav_disturbance.z = 0.05
            t.uav_disturbance_pub.publish(uav_disturbance)

        elif ((now >= 30) and (now <= 45)):
            usv_disturbance.x = -5.0
            usv_disturbance.y = 0.0
            usv_disturbance.theta = 0.0
            t.usv_disturbance_pub.publish(usv_disturbance)
            uav_disturbance.x = -0.1
            uav_disturbance.y = 0.0
            uav_disturbance.z = 0.05
            t.uav_disturbance_pub.publish(uav_disturbance)

        elif ((now >= 55) and (now <= 65)):
            usv_disturbance.x = -5.0
            usv_disturbance.y = 0.0
            usv_disturbance.theta = 0.0
            t.usv_disturbance_pub.publish(usv_disturbance)
            uav_disturbance.x = -0.1
            uav_disturbance.y = 0.0
            uav_disturbance.z = 0.05
            t.uav_disturbance_pub.publish(uav_disturbance)

        else:
            usv_disturbance.x = 0.0
            usv_disturbance.y = 0.0
            usv_disturbance.theta = 0.0
            t.usv_disturbance_pub.publish(usv_disturbance)
            uav_disturbance.x = 0.0
            uav_disturbance.y = 0.0
            uav_disturbance.z = 0.0
            t.uav_disturbance_pub.publish(uav_disturbance)

        rate.sleep()
        #rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
