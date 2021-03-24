#!/usr/bin/env python

import os
import time
import rospy
import rosbag
import math
import scipy.io as sio
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float64

#Variable Speed and Variable Heading
#Experiment 1 velocity profile for data driven optimization
class Test:
    def __init__(self):
        self.testing = True

        self.currents_pub = rospy.Publisher("/usv_currents", Pose2D, queue_size=10)
        self.cur = Pose2D()

    def desired(self, _cur):
        self.cur.x = _cur
        self.cur.y = 0
        self.cur.theta = 0.0
        self.currents_pub.publish(self.cur)

def main():
    rospy.init_node('currents', anonymous=False)
    rate = rospy.Rate(100)
    t = Test()
    dir_name = os.path.dirname(__file__)
    currents = sio.loadmat(dir_name + '/mat/current.mat')
    currents = currents['cad']
    #bag = rosbag.Bag(dir_name + '/mat/currents.bag','w')
    if t.testing:
        start_time = rospy.Time.now().secs
        i = 0
        while (not rospy.is_shutdown()) and (i < len(currents)):
            curr = currents[i]*1.5
            #bag.write('curr', curr)
            t.desired(curr)
            i = i + 1
            rate.sleep()
        bag.close()
        t.desired(0)
        t.testing = False
        rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass