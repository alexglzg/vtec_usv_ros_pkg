#!/usr/bin/env python

import os
import time
import rospy
import rosbag
import math
import scipy.io as sio
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float64, UInt8

#Variable Speed and Variable Heading
#Experiment 1 velocity profile for data driven optimization
class Test:
    def __init__(self):
        self.testing = True

        self.flag = 0
        self.arduino = 0

        self.traj = Pose2D()
        self.trajder = Pose2D()

        rospy.Subscriber("/arduino_br/ardumotors/flag", UInt8, self.flag_callback)
        rospy.Subscriber("arduino", UInt8, self.arduino_callback)

        self.d_traj_pub = rospy.Publisher("/mission/trajectory", Pose2D, queue_size=10)
        self.d_trajder_pub = rospy.Publisher("/mission/trajectory_derivative", Pose2D, queue_size=10)

    def flag_callback(self, _flag):
        self.flag = _flag

    def arduino_callback(self, _arduino):
        self.arduino = _arduino

    def desired(self, _xd, _yd, _xddot, _yddot, _start):
        self.traj.x = _xd
        self.traj.y = _yd
        self.traj.theta = _start
        self.trajder.x = _xddot
        self.trajder.y = _yddot
        self.d_traj_pub.publish(self.traj)
        self.d_trajder_pub.publish(self.trajder)

def main():
    rospy.init_node('heterogeneous_pool_infinity_trajectory', anonymous=False)
    rate = rospy.Rate(100)
    t = Test()
    dir_name = os.path.dirname(__file__)
    xd = sio.loadmat(dir_name + '/mat/traj2/pool_xd.mat')
    xd = xd['data']
    yd = sio.loadmat(dir_name + '/mat/traj2/pool_yd.mat')
    yd = yd['data']
    xddot = sio.loadmat(dir_name + '/mat/traj2/pool_xddot.mat')
    xddot = xddot['data']
    yddot = sio.loadmat(dir_name + '/mat/traj2/pool_yddot.mat')
    yddot = yddot['data']
    time.sleep(8)
    if t.testing:
        start_time = rospy.Time.now().secs
        i = 0
        while (not rospy.is_shutdown()) and (i < xd.shape[1]):
            x = (xd[0,i] - 2)*2.5/3 + 1
            y = (yd[0,i] + 12)*7/10 - 8
            xdot = (xddot[0,i])*2.5/3
            ydot = yddot[0,i]*7/10
            if i < 2:
                s = 1
            else:
                s = 2
            t.desired(x,y,xdot,ydot,s)
            i = i + 1
            rate.sleep()
        t.desired(0,0,0,0,0)
        t.testing = False
        rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
