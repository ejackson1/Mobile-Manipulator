#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rospy
from base_package.msg import jack_reset

class LimitSwitches:
    def __init__(self, init_node=False):

        if init_node:
            rospy.init_node("limit_switches", anonymous=True)
        self.rate = rospy.Rate(50) # 50Hz

        self.channels = [23, 22, 21] # left, back, right

        self.prev_sw = [1,1,1]
        self.reset = jack_reset()

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.channels[0], GPIO.IN, pull_up_down=GPIO.PUD_UP) # TODO: pull up or down?
        GPIO.setup(self.channels[1], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.channels[2], GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # GPIO.add_event_detect(self.channels[0], GPIO.BOTH, callback=self.event_callback, bouncetime=100)
        # GPIO.add_event_detect(self.channels[1], GPIO.BOTH, callback=self.event_callback, bouncetime=100)
        # GPIO.add_event_detect(self.channels[2], GPIO.BOTH, callback=self.event_callback, bouncetime=100)

        self.switches_publisher = rospy.Publisher("/base/elevator_resets", jack_reset, queue_size=10)

        self.set_reset_msg([GPIO.input(channel) for channel in self.channels])
        self.switches_publisher.publish(self.reset) # first publish is that the switches are pressed
        
        print("limit switches ready")

        while not rospy.is_shutdown(): # polling loop
            sw = [GPIO.input(channel) for channel in self.channels]
            if sw != self.prev_sw:
                self.set_reset_msg(sw)
                self.switches_publisher.publish(self.reset)
                self.prev_sw = sw

            self.rate.sleep()

    # def event_callback(self, channel):
    #     # print("channel " + str(channel) + " hit")
    #     self.channel_values[self.channels.index(channel)] = GPIO.input(channel)
    #     self.set_reset_msg()
    #     self.switches_publisher.publish(self.reset)

    def set_reset_msg(self, sw):
        self.reset.reset_left = not sw[0]
        self.reset.reset_back = not sw[1]
        self.reset.reset_right = not sw[2]

if __name__ == "__main__":
    l = LimitSwitches(init_node=True)
