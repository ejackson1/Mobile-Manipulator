#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import sys
import Encoder
import rospy
from std_msgs.msg import Int64
from base_package.msg import effort_list, encoder_values, jack_reset


class JackEncoderReader:
    def __init__(self, init_node=False):
        
        if init_node:
            rospy.init_node("jack_encoder_readings", anonymous=True)
        self.rate = rospy.Rate(10) # 10Hz
        
        self.jackEncoders = rospy.Publisher("/base/elevator_encoders", encoder_values, queue_size=10)
        # Jack Right A  = JRa
        JRa = 27
        JRb = 22
        JBa = 17
        JBb = 18
        JLa = 24
        JLb = 23

        self.encR = Encoder.Encoder(JRa, JRb)
        self.encL = Encoder.Encoder(JLa, JLb)
        self.encB = Encoder.Encoder(JBa, JBb)

        self.encR_zero, self.encL_zero, self.encB_zero = 0, 0, 0

        self.encoder_writer = rospy.Subscriber("/base/elevator_resets", jack_reset, self.encoder_reset)

    def encoder_reset(self, msg):
        if msg.reset_left:
            self.encL_zero = self.encL.read()
        if msg.reset_back:
            self.encB_zero = self.encB.read()
        if msg.reset_right:
            self.encR_zero = self.encR.read()
        
    def main(self):
        # if home_config == True:
        #     home_config()
        values = encoder_values()
        print("publishing values")
        
        while not rospy.is_shutdown():
            # read encoder values relative to their zeroed value
            rightJack = self.encR.read() - self.encR_zero 
            leftJack = self.encL.read() - self.encL_zero 
            backJack = self.encB.read() - self.encB_zero 
            
            # rostopic publishing
            values.Values = [int(leftJack), int(backJack), int(rightJack)]
            self.jackEncoders.publish(values)
            self.rate.sleep()


if __name__ == "__main__":
    j = JackEncoderReader(init_node=True)
    j.main()
