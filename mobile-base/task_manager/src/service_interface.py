#!/usr/bin/env python3

import rospy
from abstract_arm.moveItClient import MoveitArmClient
from task_manager.msg import task

class ServiceInterface:
    def __init__(self, init_node=False):
        if init_node:
            rospy.init_node("service_interface", anonymous=True)

        self.arm_services = MoveitArmClient()
        self.t = task()

        #TODO: base services

    def move_arm(self, task_type, orientation, gripper):
        if task_type == self.t.TASK_PICK:
            self.arm_services.move_arm_EE(orientation.pose) #NOTE: we're throwing the poseStamped header away
        else:
            self.arm_services.move_arm_angles(orientation)

        self.arm_services.move_gripper(0.0 if gripper else 100.0) #TODO: double check gripper values

        #TODO: return true if successful?

    def home_arm(self):
        self.arm_services.send_arm_home()
    
    def is_arm_in_home_pos(self):
        return self.arm_services.is_arm_in_home_pos()