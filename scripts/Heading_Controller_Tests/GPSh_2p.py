#!/usr/bin/env python

import os
import time
import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D

#Constant Speed and Constant Heading
#GPS bearing rads
class Test:
    def __init__(self):
        self.testing = True

        self.dt = 0
        self.dh = 0
        self.distance = 0
        self.bearing = 0

        self.latitude = 0
        self.longitude = 0
        self.yaw = 0

        rospy.Subscriber("ins_pose", Pose2D, self.gps_callback)

        self.d_thrust_pub = rospy.Publisher("desired_thrust", Float64, queue_size=10)
        self.d_heading_pub = rospy.Publisher("desired_heading", Float64, queue_size=10)
        self.waypoint_pub = rospy.Publisher("waypoint", Pose2D, queue_size=10)
        self.distance_pub = rospy.Publisher("distance", Float64, queue_size=10)

    def gps_callback(self, gps):
        self.latitude = gps.x
        self.longitude = gps.y
        self.yaw = gps.theta

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

        tx = 20
        if self.distance < 5:
            tx = 15
        if self.distance < 1.5:
            tx = 0
            self.bearing = self.yaw
            self.testing = False

        self.desired(tx, self.bearing)

        waypoint = Pose2D()
        waypoint.x = latitude2
        waypoint.y = longitud2
        waypoint.theta = self.bearing
        self.waypoint_pub.publish(waypoint)
        self.distance_pub.publish(self.distance)

        time.sleep(0.1)


    def desired(self, thrust, heading):
        self.dh = heading
        self.dt = thrust
        self.d_heading_pub.publish(self.dh)
        self.d_thrust_pub.publish(self.dt)

def main():
    rospy.init_node('GPSh_2p', anonymous=True)
    t = Test()
    while t.testing: 
        t.get_degrees_and_distance_to_gps_coords(25.653332, -100.291456)
    t.testing = True
    while t.testing:
        t.get_degrees_and_distance_to_gps_coords(25.653338, -100.2911589)
    rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
