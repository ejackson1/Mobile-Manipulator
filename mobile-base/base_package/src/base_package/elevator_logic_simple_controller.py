#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from base_package.msg import effort_list, encoder_values, jack_reset

class ElevatorNode:
    def __init__(self, init_node=False):

        if init_node:
            rospy.init_node("elevator_logic", anonymous=True)

        # Listen to the xbox controller!
        rospy.Subscriber("/joy", Joy, self.sendEfforts)

        # TODO
        # Listen to reported encoder values
        rospy.Subscriber("/base/elevator_encoders", encoder_values, self.updatePosition)
        self.efforts = effort_list()

        # Publish calculated efforts to low level controllers
        self.elevator_efforts = rospy.Publisher("/base/elevator_efforts", effort_list, queue_size=10)
        self.elevator_resets = rospy.Publisher("/base/elevator_resets", jack_reset, queue_size=10)

        # Publish updates to Simulation
        # arm_name = rospy.get_param("arm/name")
        # arm_namespace = "panda" if arm_name=="panda" else "my_gen3"
        # arm_controller_str = "/" + arm_namespace + "/" + arm_name + "_{0}_joint_controller/command"

        # self.zInsertController = rospy.Publisher(arm_controller_str.format('z'), Float64, queue_size=10)

    def sendEfforts(self, msg):
        # helper function to send Effort values to cim motors
        # msg [Joy]

        speed = 100 if msg.buttons[6] == 1 else 20 # right trigger = fast mode

        left_jack = speed*(msg.buttons[0] + -msg.buttons[1]) # B up A down
        back_jack = speed*(msg.buttons[4] + -msg.buttons[5]) # RB up LB down
        right_jack = speed*(msg.buttons[2] + -msg.buttons[3]) # Y up X down

        if msg.axes[7]: # D-pad up/down
            self.efforts.Efforts = [-speed*el for el in [msg.axes[7]]*3] # moves all 3 jacks
        else: 
            self.efforts.Efforts = [left_jack, back_jack, right_jack] # moves each jack individually
        
        # send signal to reset encoders to 0
        if msg.buttons[7]:
            print("resetting encoders")
            j = jack_reset()
            j.reset_left, j.reset_back, j.reset_right = 1, 1, 1
            self.elevator_resets.publish(j)

        # add feedback loop to ensure all motors reach the same height
        self.elevator_efforts.publish(self.efforts)


    def updatePosition(self, msg):
        # TODO
        # Publish rough coordinates of elevator to update in sim!

        debug_str = "curr encoders: " + str(msg.Values)
        
        print(debug_str)


if __name__=="__main__":
    e = ElevatorNode(init_node=True)
    rospy.spin()

    kill_efforts = effort_list()
    kill_efforts.Efforts = [0.0, 0.0, 0.0]
    e.elevator_efforts.publish(kill_efforts)
