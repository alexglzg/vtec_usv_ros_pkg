#!/usr/bin/env python

import os
import time
import serial
import rospy
import math
from std_msgs.msg import Float64

class Motors:
    def __init__(self):
        self.thrust = True

        self.previousRightMotorValue = 0
        self.previousLeftMotorValue = 0

        self.minRotationS = 300 #rev/min
        self.maxRotationS = 300 #rev/min

        self.maxPowerValue = 1900
        self.minPowerValue = 1100

        self.powerR = 0 
        self.powerL = 0 

        self.thrusterInitPosition = 1500
        self.thrustersBack  = 'b'
        self.thrustersFront = 'f'
        #self.servoInitPosition = 90
        self.baudRate = 115200
        #self.serial_port = '/dev/ttyACM0'
        self.serial_port = '/dev/ttyUSB1'

        rospy.Subscriber("right_thruster", Float64, self.right_callback)
        rospy.Subscriber("left_thruster", Float64, self.left_callback)

        #serial communication Handler
        self.ser = serial.Serial(self.serial_port, self.baudRate)
        time.sleep(1)

    def right_callback(self, right_t):
        self.powerR = right_t.data
        #rospy.logwarn(self.powerR)

    def left_callback(self, left_t):
        self.powerL = left_t.data
        #rospy.logwarn(self.powerL)

    #format value to proper length
    def check_value_size(self, val):
        '''
        @desc     format value to proper length
        @params string
        @return string
        '''
        if len(val) == 4:
            return val 
        elif len(val) == 3:
            return '0' + val
        elif len(val) == 2:
            return '00' + val 
        elif len(val) == 1:
            return '000' + val 
    
    def move_thrusters(self,powerR=1500, powerL=1500):
        #validate the pwm range
        if powerR < 1100 or powerR > 1900 or powerL < 1100 or powerL > 1900:
            rospy.logwarn("Thruster power must be between 1100 - 1900")
        else:
            #Format motors value
            pR = str(powerR)
            #pR = self.check_value_size(pR)
            pL = str(powerL)
            #pL = self.check_value_size(pL)    
            val = '%' + 'B,' + pR + ',' + pL + '%'

            #Send motors value to arduino
            #value = bytearray(val)
            self.ser.write(val)
            
            self.ser.flush()

            #print(self.ser.isOpen())

            #Debug response
            #print(self.ser.read(self.ser.inWaiting()).decode())
            #print('value: ', val)

    def move(self, powerR=0,powerL=0):
        #validate the pwm range
        if powerR < -400 or powerR > 400 or powerL < -400 or powerL > 400:
            rospy.logwarn("The power is not on the correct range")
        else:
            realPowerValueR = round(powerR + 1500)
            realPowerValueL = round(powerL + 1500)
            self.move_thrusters(realPowerValueR,realPowerValueL)
            #print('moving')
            #while(utility.previousLeftMotorValue != powerL or utility.previousRightMotorValue != powerR):

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
    rospy.init_node('motors', anonymous=True)
    rate = rospy.Rate(50) # 50hz
    m = Motors()
    while not rospy.is_shutdown():    
        while m.thrust:    
            m.run(m.powerR, m.powerL)
            rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
