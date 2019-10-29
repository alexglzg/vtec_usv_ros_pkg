#!/usr/bin/env python

import os
import time
import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D

#Constant Speed and Constant Heading
#0.5 m/s (until short distance) and GPS bearing rads
class Test:
    def __init__(self):
        self.testing = True

        self.ds = 0
        self.dh = 0
        self.distance = 0
        self.bearing = 0

        self.NEDx = 0
        self.NEDy = 0
        self.yaw = 0

        self.dmax = 10
        self.dmin = 2
        self.gamma = 0.003

        self.k = 1

        self.Waypointpath = Pose2D()
        self.LOSpath = Pose2D()

        rospy.Subscriber("NED_pose", Pose2D, self.gps_callback)

        self.d_speed_pub = rospy.Publisher("desired_speed", Float64, queue_size=10)
        self.d_heading_pub = rospy.Publisher("desired_heading", Float64, queue_size=10)
        self.waypoint_pub = rospy.Publisher("waypoint", Pose2D, queue_size=10)
        self.LOS_pub = rospy.Publisher("LOS", Pose2D, queue_size=10)

    def gps_callback(self, gps):
        self.NEDx = gps.x
        self.NEDy = gps.y
        self.yaw = gps.theta

    def LOSloop(self, listvar):
        if self.k < len(listvar)/2:
            x1 = listvar[2*self.k - 2]
            y1 = listvar[2*self.k - 1]
            x2 = listvar[2*self.k]
            y2 = listvar[2*self.k + 1]
            self.Waypointpath.x = x2
            self.Waypointpath.y = y2
            self.waypoint_pub.publish(self.Waypointpath)
            xpow = math.pow(x2 - self.NEDx, 2)
            ypow = math.pow(y2 - self.NEDy, 2)
            self.distance = math.pow(xpow + ypow, 0.5)
            if self.distance > 2:
                self.LOS(x1, y1, x2, y2)
            else:
                self.k += 1
        else:
            self.testing = False

    def LOS(self, x1, y1, x2, y2):
        ak = math.atan2(y2-y1,x2-x1)
        ye = -(self.NEDx - x1)*math.sin(ak) + (self.NEDy - y1)*math.cos(ak)
        xe = (self.NEDx - x1)*math.cos(ak) + (self.NEDy - y1)*math.sin(ak)
        delta = (self.dmax - self.dmin)*math.exp(-(1/self.gamma)*math.fabs(ye)) + self.dmin
        psi_r = math.atan(-ye/delta)
        self.bearing = ak + psi_r

        if (math.fabs(self.bearing) > (math.pi)):
            self.bearing = (self.bearing/math.fabs(self.bearing))*(math.fabs(self.bearing)-2*math.pi)

        xlos = x1 + (delta+xe)*math.cos(ak)
        ylos = y1 + (delta+xe)*math.sin(ak)
        self.LOSpath.x = xlos
        self.LOSpath.y = ylos
        self.LOS_pub.publish(self.LOSpath)

        self.vel = 1.4
        #if self.distance < 6:
        #    self.vel = 0.4

        self.desired(self.vel, self.bearing)

    def desired(self, speed, heading):
        self.dh = heading
        self.ds = speed
        self.d_heading_pub.publish(self.dh)
        self.d_speed_pub.publish(self.ds)

def main():
    rospy.init_node('LOS2sh', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    t = Test()
    a = [2,2,6,-15,1,-25,6,-35]
    t.k = 1
    t.Waypointpath.x = a[0]
    t.Waypointpath.y = a[1]
    time.sleep(1)
    t.waypoint_pub.publish(t.Waypointpath)
    time.sleep(1)
    while not rospy.is_shutdown() and t.testing:
        t.LOSloop(a)
        rate.sleep()
    t.desired(0,t.yaw)
    rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
