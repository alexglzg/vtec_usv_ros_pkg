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

        rospy.Subscriber("NED_pose", Pose2D, self.gps_callback)

        self.d_speed_pub = rospy.Publisher("desired_speed", Float64, queue_size=10)
        self.d_heading_pub = rospy.Publisher("desired_heading", Float64, queue_size=10)

    def gps_callback(self, gps):
        self.NEDx = gps.x
        self.NEDy = gps.y
        self.yaw = gps.theta

    def LOS(self, x1, y1, x2, y2):
        ak = math.atan2(y2-y1,x2-x1)
        ye = -(self.NEDx - x1)*math.sin(ak) + (self.NEDy - y1)*math.cos(ak)
        delta = (self.dmax - self.dmin)*math.exp(-self.gamma*math.fabs(ye)) + self.dmin
        psi_r = math.atan(-ye/delta)
        self.bearing = ak + psi_r
        if (math.fabs(self.bearing) > (math.pi)):
            self.bearing = (self.bearing/math.fabs(self.bearing))*(math.fabs(self.bearing)-2*math.pi)
        xpow = math.pow(x2 - self.NEDx, 2)
        ypow = math.pow(y2 - self.NEDy, 2)
        self.distance = math.pow(xpow + ypow, 0.5)

        self.vel = 0.7
        if self.distance < 6:
            self.vel = 0.4
        if self.distance < 2:
            self.vel = 0
            self.bearing = self.yaw
            self.testing = False

        self.desired(self.vel, self.bearing)

    def get_degrees_and_distance_to_gps_coords(self,latitude2, longitud2):
        latitude1 = self.latitude
        longitud1 = self.longitude

        longitud_distance = (longitud1 - longitud2)
        y_distance = math.sin(longitud_distance) * math.cos(latitude2)
        x_distance = math.cos(latitude1) * math.sin(latitude2) - math.sin(latitude1) * math.cos(latitude2) * math.cos(longitud_distance)
        self.bearing = math.atan2(y_distance, x_distance)
        self.bearing = self.bearing * (-1)
        self.deg = math.degrees(self.bearing)
        rospy.logwarn("bearing %f", self.deg)

        phi1 = math.radians(latitude1)
        phi2 = math.radians(latitude2)
        dphi = math.radians(latitude2 - latitude1)
        dlam = math.radians(longitud2 - longitud1)
        a = math.sin(dphi/2)*math.sin(dphi/2) + math.cos(phi1)*math.cos(phi2)* math.sin(dlam/2)*math.sin(dlam/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        self.distance = 6378137 * c
        rospy.logwarn("distance %f", self.distance)

        self.vel = 0.7
        if self.distance < 2:
            self.vel = 0
            self.bearing = self.yaw
            self.testing = False

        self.desired(self.vel, self.bearing)

    def desired(self, speed, heading):
        self.dh = heading
        self.ds = speed
        self.d_heading_pub.publish(self.dh)
        self.d_speed_pub.publish(self.ds)

def main():
    rospy.init_node('LOSsh', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    t = Test()
    while not rospy.is_shutdown() and t.testing:
        t.LOS(2,2,6,-15)
        rate.sleep()
    rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
