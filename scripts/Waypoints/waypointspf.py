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
        self.d_speed_pub = rospy.Publisher("/guidance/desired_speed", Float64, queue_size=10)

    def desired(self, path, speed):
      self.path_pub.publish(path)
      self.d_speed_pub.publish(speed)

def main():
    rospy.init_node('waypoints', anonymous=False)
    rate = rospy.Rate(20) # 100hz
    t = Test()
    path_array = Float32MultiArray()
    path_array.layout.data_offset = 5
    path_array.data = [0, 4, 45, 4, 0]
    speed = Float64()
    speed.data = 0.5
    while not rospy.is_shutdown() and t.testing:
        time.sleep(1)
        start_time = rospy.Time.now().secs
        while (rospy.Time.now().secs - start_time) <= 50 and not rospy.is_shutdown():
            t.desired(path_array, speed)
            rate.sleep()
        speed.data = 0.0
        t.desired(path_array, speed)
        t.testing = False
        rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
