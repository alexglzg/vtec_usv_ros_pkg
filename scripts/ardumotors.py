#!/usr/bin/env python

import os
import time
#import serial
import rospy
import math
from std_msgs.msg import Float64
from std_msgs.msg import UInt16

class Motors:
    def __init__(self):
        self.thrust = True

        self.maxPowerValue = 1900
        self.minPowerValue = 1100

        self.powerR = 0 
        self.powerL = 0 

        rospy.Subscriber("right_thruster", Float64, self.right_callback)
        rospy.Subscriber("left_thruster", Float64, self.left_callback)

        self.r_pwm_pub = rospy.Publisher("rpwm", UInt16, queue_size=10)
        self.l_pwm_pub = rospy.Publisher("lpwm", UInt16, queue_size=10)

    def right_callback(self, right_t):
        self.powerR = right_t.data
        #rospy.logwarn(self.powerR)

    def left_callback(self, left_t):
        self.powerL = left_t.data
        #rospy.logwarn(self.powerL)

    def move_thrusters(self,powerR=1500, powerL=1500):
        #validate the pwm range
        if powerR < 1100 or powerR > 1900 or powerL < 1100 or powerL > 1900:
            rospy.logwarn("Thruster power must be between 1100 - 1900")
        else:
            pwmR = powerR
            pwmL = powerL
            self.r_pwm_pub.publish(pwmR)
            self.l_pwm_pub.publish(pwmL)

    def newtons(self, powerR=0, powerL=0):
        #validate the Newtons range
        if (powerR < -30 or powerR > 36.5 or powerL < -30 or powerL > 36.5):
            rospy.logwarn("Saturation range")
            realPowerValueR = powerR * 0.5
            realPowerValueL =  powerL * 0.5

        else:
            if (powerR >= 0):
                realPowerValueR = round((powerR / 36.5 * 385)+1515)
            elif (powerR < 0):
                realPowerValueR = round((powerR / 30 * 385)+1485)
            if (powerL <= 0):
                realPowerValueL = round((powerL / 30 * 385)+1485)
            elif(powerL > 0):
                realPowerValueL = round((powerL / 36.5 * 385)+1515)

        self.move_thrusters(realPowerValueR,realPowerValueL)
        #print('moving')

    def run(self, powerR = 0, powerL = 0):
        self.newtons(powerR, powerL)

def main():
    rospy.init_node('ardumotors', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    m = Motors()
    while not rospy.is_shutdown() and m.thrust:
        #if m.thrust:
        m.run(m.powerR, m.powerL)
        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
