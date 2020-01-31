#!/usr/bin/env python

import numpy as np

import os
import time
import math

import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray

NODE_NAME_THIS = 'rl_control'

class RlControl:


    def __init__(self):


        self.w1 = 0
        self.w2 = 0
        self.w3 = 0
        self.b1 = 0
        self.b2 = 0
        self.b3 = 0

        self.NEDx = 0
        self.NEDy = 0
        self.yaw = 0

        self.u = 0 #surge speed
        self.v = 0 #sway speed
        self.r = 0 #yaw rate

        self.waypoint_array = []
        self.current_waypoint_array = []
        
        self.c = 1.27

        self.counter=0

        self.testing = True

#Controller outputs
        self.T_port = 0 #Thrust in Newtons
        self.T_stbd = 0 #Thrust in Newtons

#IMU data subscribers
        rospy.Subscriber("local_vel", Vector3, self.local_vel_callback)
        rospy.Subscriber("NED_pose", Pose2D, self.gps_callback)
#Waypoints subscriber
        rospy.Subscriber("/mission/waypoints", Float32MultiArray, self.waypoints_callback)

#Thruster data publishers
        self.right_thruster_pub = rospy.Publisher("right_thruster", Float64, queue_size=10)
        self.left_thruster_pub = rospy.Publisher("left_thruster", Float64, queue_size=10)

    def local_vel_callback(self, upsilon):
        self.u = upsilon.x
        self.v = upsilon.y
        self.r = upsilon.z

    def gps_callback(self, gps):
        self.NEDx = gps.x
        self.NEDy = gps.y
        self.yaw = gps.theta

    def waypoints_callback(self, msg):
        waypoints = []
        leng = (msg.layout.data_offset)

        for i in range(int(leng)):
            waypoints.append(msg.data[i])
        self.waypoint_array = waypoints

    def load_weights(self,path):
        loaded = np.load(path)
        self.w1=loaded['w1']
        self.b1=loaded['b1']
        self.w2=loaded['w2']
        self.b2=loaded['b2']
        self.w3=loaded['w3']
        self.b3=loaded['b3']

    def waypoint_deploy(self, waypoint_array):
        if self.counter <= len(waypoint_array):
            x_desired = waypoint_array[self.counter - 1]
            y_desired = waypoint_array[self.counter]
            x_distance = math.pow(x_desired - self.NEDx, 2)
            y_distance = math.pow(y_desired - self.NEDy, 2)
            distance = math.pow(x_distance + y_distance, 0.5)
            if distance > 0.5:
                self.control(x_desired,y_desired)
            else:
                self.counter+=2
        else:
            self.desired_thrust(0, 0)

    def control(self,x_desired,y_desired):
        state=np.array([self.NEDx,self.NEDy,self.yaw,self.u,self.v,self.r])
        state[0] -= x_desired
        state[1] -= y_desired
        action = self.run_numpy_action_network(state)
        self.desired_thrust(action[0,0],action[0,1])

    def run_numpy_action_network(self, state):
        h1 = np.tanh(np.matmul(state, self.w1) + self.b1)
        h2 = np.tanh(np.matmul(np.concatenate([state, h1[0]]), self.w2) + self.b2)
        action = np.matmul(np.concatenate([state, h1[0], h2[0]]), self.w3) + self.b3
        return action

    def desired_thrust(self,T_port,T_stbd):
        self.right_thruster_pub.publish(T_stbd)
        self.left_thruster_pub.publish(T_port)

def main():
    rospy.init_node(NODE_NAME_THIS, anonymous=False)
    rate = rospy.Rate(100) # 100hz
    rospy.loginfo("Test node running")
    rl_control = RlControl()
    rl_control.load_weights('/home/ubuntu/catkin_ws/src/sensors/scripts/weights/'+'weights.npz')
    while not rospy.is_shutdown() and rl_control.testing:
        if rl_control.current_waypoint_array != rl_control.waypoint_array:
            rl_control.counter = 1
            rl_control.current_waypoint_array = rl_control.waypoint_array
        if len(rl_control.current_waypoint_array) > 1:
            rl_control.waypoint_deploy(rl_control.current_waypoint_array)
            rate.sleep()
    rl_control.desired_thrust(0,0)
    rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
