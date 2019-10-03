#!/usr/bin/env python
#Backstepping with PI speed controller and heading controller

import os
import time
import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3

class Controller:
    def __init__(self):
        self.activated = True #determines if the controller should run

#Controller hydrodynamic and physical constants
        self.X_u_dot = -2.25
        self.Y_v_dot = -23.13
        self.mass = 30
        self.X_uu = 0
        self.X_u = 0
        self.N_r = 0
        self.N_r_dot = -2.79
        self.I_z = 4.1
        self.B = 0.41

#Controller gains
        self.kp = -1.5 #PI speed controller P gain
        self.ki = -0.5 #PI speed controller I gain
        self.k1 = -1.8 #Heading controller yaw error gain
        self.k2 = 10.8 #Heading controller yaw rate gain

#Desired surge speed and heading
        self.u_d = 0
        self.psi_d = 0

#Controller feedback variables
        self.u = 0 #surge speed
        self.v = 0 #sway speed
        self.r = 0 #yaw rate speed
        self.psi = 0 #yaw

#Controller internal variables
        self.T_x = 0
        self.epsilon_u = 0
        self.error = 0
        self.error_i = 0
        self.X_drag = 0
        self.T_z = 0
        self.epsilon_psi = 0

#Controller outputs
        self.T_port = 0 #Thrust in Newtons
        self.T_stbd = 0 #Thrust in Newtons

#Desired values subscribers
        rospy.Subscriber("desired_speed", Float64, self.dspeed_callback)
        rospy.Subscriber("desired_heading", Float64, self.dheading_callback)

#IMU data subscribers
        rospy.Subscriber("local_vel", Vector3, self.local_vel_callback)
        rospy.Subscriber("ins_pose", Pose2D, self.ins_pose_callback)
        rospy.Subscriber("ins_ar", Vector3, self.ins_ar_callback)        

#Thruster data publishers
        self.right_thruster_pub = rospy.Publisher("right_thruster", Float64, queue_size=10)
        self.left_thruster_pub = rospy.Publisher("left_thruster", Float64, queue_size=10)        

    def dspeed_callback(self, d_speed):
        self.u_d = d_speed.data

    def dheading_callback(self, d_heading):
        self.psi_d = d_heading.data

    def local_vel_callback(self, upsilon_xy):
        self.u = upsilon_xy.x
        self.v = upsilon_xy.y

    def ins_pose_callback(self, pose):
        self.psi = pose.theta

    def ins_ar_callback(self, upsilon_z):
        self.r = upsilon_z.z

    def control(self, u_d=0, psi_d=0):

#Nr hydrodynamic variable
        self.N_r = (-0.52)*(math.pow(math.pow((self.u), 2) + math.pow((self.v), 2), 0.5))
#Xu and Xuu
        if self.u > 1.2:
            self.Xu = 64.55
            self.Xuu = -70.92
        if self.u < 1.2:
            self.Xu = 25
            self.Xuu = 0

        self.error_i = self.error #Error for integral
        self.error = self.u - u_d #Error
        rospy.logwarn("u error %f", self.error)
        self.epsilon_u = (self.kp)*(self.error) + (self.ki)*(self.error - self.error_i)

        self.epsilon_psi = (self.k1)*(self.psi - psi_d) - (self.k2)*(self.r)
        degree_psi = math.degrees(self.psi)
        degree_d = math.degrees(psi_d)
        rospy.logwarn("psi error %f", degree_psi - degree_d)

        self.X_drag = (self.X_uu)*(self.u)*math.fabs(self.u) + (self.X_u)*(self.u)
        self.T_x = (self.mass - self.X_u_dot)*(self.epsilon_u) - (self.mass - self.Y_v_dot)*(self.v)*(self.r) - (self.X_drag)

        self.T_z = (self.I_z - self.N_r_dot)*(self.epsilon_psi) - (self.X_u_dot*(-1) + self.Y_v_dot)*(self.u)*(self.v) - (self.N_r)*(self.r)

        self.T_port = (self.T_x/2) + (self.T_z/self.B)
        self.T_stbd = (self.T_x/2) - (self.T_z/self.B)


#Controller outputs
        self.right_thruster_pub.publish(self.T_stbd)
        self.left_thruster_pub.publish(self.T_port)

    def run(self, u_d=0, psi_d=0):
        self.control(u_d, psi_d)

def main():
    rospy.init_node('controller', anonymous=True)
    c = Controller()
    while c.activated:
        c.run(c.u_d, c.psi_d)
        rospy.logwarn("control")
        time.sleep(1)

if __name__ == "__main__":
    main()