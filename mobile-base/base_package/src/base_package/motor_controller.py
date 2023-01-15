#!/usr/bin/env python3

import adafruit_pca9685 as ada
import board
import time
import rospy
from base_package.msg import effort_list, jack_reset

# Controls motors via i2c on raspberry pi - must be launched remotely
class MotorController:
    def __init__(self, init_node=False):

        if init_node:
            rospy.init_node("motor_controller", anonymous=True)

        # setup i2c
        i2c_pwm = board.I2C()
        self.pwm = ada.PCA9685(i2c_pwm)

        period = .012
        self.pwm.frequency = 1/period # period = 10ms ish

        self.jack_resets = [False, False, False]
        
        # setup topic listeners
        rospy.Subscriber("/base/mecanum_efforts", effort_list, self.spin_wheels)
        rospy.Subscriber("/base/elevator_efforts", effort_list, self.spin_elevator_motors)
        rospy.Subscriber("/base/elevator_resets", jack_reset, self.update_resets)


    # drive callback
    def spin_wheels(self, msg):
        for i in range(4):
            self.set_motor_speed(i, msg.Efforts[i])
    
    # jack callback
    def spin_elevator_motors(self, msg):
        print("received elevator effort: " + str(msg.Efforts) + ", jacks: " + str(self.jack_resets))
        for i in range(3):
            if not (self.jack_resets[i] and msg.Efforts[i] > 0): # if reset, prevent motors from going down
                self.set_motor_speed(i+4, msg.Efforts[i])
            else:
                self.set_motor_speed(i+4, 0)

    def update_resets(self, msg):
        self.jack_resets = [msg.reset_left, msg.reset_back, msg.reset_right]

    def translate(self, value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)

    # -100 to 100 input
    def set_motor_speed(self, channel, percent_speed):
        # MAX_DUTY = 0xFFFF
        # high_time = translate(percent_speed, -100, 100, .001, .002)
        # cycle = high_time*pwm.frequency*MAX_DUTY

        # backwards: 5370
        # stop: 8165
        # forwards: 10960
        cycle = self.translate(percent_speed, -100, 100, 5370, 10960) # found by guess and check

        self.pwm.channels[channel].duty_cycle = int(cycle) # 0 to 65535

# motor testing
if __name__ == "__main__":
    m = MotorController(init_node=True)
    rospy.spin()

    # if interrupted, stop all motors
    for i in range(7):
        m.set_motor_speed(i, 0)
