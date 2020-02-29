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

NODE_NAME_THIS = 'rl_u_psi'

model = rospy.get_param("rl_u_psi/model")

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

        self.u_d = 0
        self.psi_d = 0
        
        self.c = 1.27

        self.counter = 0

        self.testing = True


#Controller outputs
        self.T_port = 0 #Thrust in Newtons
        self.T_stbd = 0 #Thrust in Newtons

#IMU data subscribers
        rospy.Subscriber("local_vel", Vector3, self.local_vel_callback)
        rospy.Subscriber("NED_pose", Pose2D, self.gps_callback)
#Desired subscriber
        rospy.Subscriber("desired_speed", Float64, self.dspeed_callback)
        rospy.Subscriber("desired_heading", Float64, self.dheading_callback)


#Thruster data publishers
        self.right_thruster_pub = rospy.Publisher("right_thruster", Float64, queue_size=10)
        self.left_thruster_pub = rospy.Publisher("left_thruster", Float64, queue_size=10)
	self.u_error_pub = rospy.Publisher("u_error", Float64, queue_size=10)
        self.psi_error_pub = rospy.Publisher("psi_error", Float64, queue_size=10)

    def local_vel_callback(self, upsilon):
        self.u = upsilon.x
        self.v = upsilon.y
        self.r = upsilon.z

    def gps_callback(self, gps):
        self.NEDx = gps.x
        self.NEDy = gps.y
        self.yaw = gps.theta

    def dspeed_callback(self, d_speed):
        self.u_d = d_speed.data

    def dheading_callback(self, d_heading):
        self.psi_d = d_heading.data

    def load_weights(self,path):
        loaded = np.load(path)
        self.w1=loaded['w1']
        self.b1=loaded['b1']
        self.w2=loaded['W2']
        self.b2=loaded['b2']
        self.w3=loaded['w3']
        self.b3=loaded['b3']


    def control(self,u_desired,psi_desired):
        eu = u_desired - self.u
        state=np.array([self.yaw,self.u,self.v,self.r,eu,self.T_port, self.T_stbd])
        state[0] -= psi_desired
        self.u_error_pub.publish(eu)
        self.psi_error_pub.publish(state[0])
        self.T_port, self.T_stbd = self.run_numpy_action_network(state)
        self.desired_thrust(self.T_port, self.T_stbd)

    def run_numpy_action_network(self, state):
        h1 = np.tanh(np.matmul(state, self.w1) + self.b1)
        h2 = np.tanh(np.matmul(np.concatenate([state, h1]), self.w2) + self.b2)
        action = np.matmul(np.concatenate([state, h1, h2]), self.w3) + self.b3
        action = np.tanh(action)*35
        if action[0] > 36.5:
            action[0] = 36.5
        elif action[0] < -30:
            action[0] = -30
        if action[1] > 36.5:
            action[1] = 36.5
        elif action[1] < -30:
            action[1] = -30
        return action[0], action[1]

    def desired_thrust(self,T_port,T_stbd):
        self.right_thruster_pub.publish(T_stbd)
        self.left_thruster_pub.publish(T_port)

def main():
    rospy.init_node(NODE_NAME_THIS, anonymous=False)
    rate = rospy.Rate(100) # 100hz
    rospy.loginfo("Test node running")
    rl_control = RlControl()
    rl_control.load_weights('/home/ubuntu/catkin_ws/src/sensors/scripts/weights/speed_heading/'+ model + '.npz')
    while not rospy.is_shutdown() and rl_control.testing:
        rl_control.control(rl_control.u_d, rl_control.psi_d)
        rate.sleep()
    rl_control.desired_thrust(0,0)
    rospy.logwarn("Finished")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
