#!/usr/bin/env python

import os
import time
import rospy
import math
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8

class Motors:
    def __init__(self):
        self.active = True

        self.max_pwm = 1900
        self.min_pwm = 1100
        self.stop_pwm = 1500

        self.max_thrust = 36.5
        self.min_thrust = -30

        self.thrust_pwm_conversion = 385
        self.initial_forward_thrust = 1515
        self.initial_reverse_thrust = 1485

        self.power_right = 0 
        self.power_left = 0 

        rospy.Subscriber("/usv_control/controller/right_thruster", Float64, self.right_callback)
        rospy.Subscriber("/usv_control/controller/left_thruster", Float64, self.left_callback)

        self.r_pwm_pub = rospy.Publisher("rpwm", UInt16, queue_size=10)
        self.l_pwm_pub = rospy.Publisher("lpwm", UInt16, queue_size=10)
        self.flag_pub = rospy.Publisher("/arduino_br/ardumotors/flag", UInt8, queue_size=10)

    def right_callback(self, right_t):
        self.power_right = right_t.data
        #rospy.logwarn(self.power_right)

    def left_callback(self, left_t):
        self.power_left = left_t.data
        #rospy.logwarn(self.power_left)

    def send_pwm(self, power_right, power_left):
        '''
        @name: send_pwm
        @brief: PWM signal topic publishing for rosserial arduino, and flag for asmc to start.
        @param: power_right: PWM value of the right thruster
                power_left: PWM value of the left thruster
        @return: --
        '''
        if power_right < self.min_pwm or power_right > self.max_pwm or power_left < self.min_pwm or power_left > self.max_pwm:
            rospy.logwarn("Thruster power must be between 1100 - 1900")
        else:
            pwm_right = power_right
            pwm_left = power_left
            self.r_pwm_pub.publish(pwm_right)
            self.l_pwm_pub.publish(pwm_left)
            flag = 1
            self.flag_pub.publish(flag)

    def newtons_to_pwm(self, power_right=0, power_left=0):
        '''
        @name: newtons_to_pwm
        @brief: Conversion of thrust value from Newtons to PWM.
        @param: power_right: Newtons value of the right thruster
                power_left: Newtons value of the left thruster
        @return: --
        '''
        pwm_right = 0
        pwm_left = 0
        if (power_right < self.min_thrust or power_right > self.max_thrust or power_left < self.min_thrust or power_left > self.max_thrust):
            rospy.logwarn("Saturation range")
            pwm_right = power_right * 0.5
            pwm_left =  power_left * 0.5

        else:
            if (power_right >= 0):
                pwm_right = round((power_right / self.max_thrust * self.thrust_pwm_conversion)+self.initial_forward_thrust)
            elif (power_right < 0):
                pwm_right = round((power_right / (-self.min_thrust) * self.thrust_pwm_conversion)+self.initial_reverse_thrust)
            if (power_left <= 0):
                pwm_left = round((power_left / (-self.min_thrust) * self.thrust_pwm_conversion)+self.initial_reverse_thrust)
            elif(power_left > 0):
                pwm_left = round((power_left / self.max_thrust * self.thrust_pwm_conversion)+self.initial_forward_thrust)

        self.send_pwm(pwm_right, pwm_left)

    def run(self, power_right = 0, power_left = 0):
        self.newtons_to_pwm(power_right, power_left)

def main():
    rospy.init_node('ardumotors', anonymous=False)
    rate = rospy.Rate(100) # 100hz
    motors = Motors()
    while not rospy.is_shutdown() and motors.active:
        motors.run(motors.power_right, motors.power_left)
        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
