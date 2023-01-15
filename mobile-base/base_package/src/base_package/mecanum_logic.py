#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Float64MultiArray
from base_package.msg import effort_list


class MecanumNode:
    def __init__(self, init_node=False):

        if init_node:
            rospy.init_node("mecanum_logic", anonymous=True)

        # Listen to the xbox controller!
        rospy.Subscriber("/joy", Joy, self.sendEfforts)

        # TODO
        # Listen to reported encoder values
        rospy.Subscriber("/base/mecanum_encoders", effort_list, self.updatePosition)

        # Publish calculated efforts to low level controllers
        self.mecanum_efforts = rospy.Publisher("/base/mecanum_efforts", effort_list, queue_size=10)

        # Publish updates to Simulation
        base_controller_str = "/base/base_{0}_joint_controller/command"
        self.xBaseController = rospy.Publisher(base_controller_str.format('x'), Float64, queue_size=10)
        self.yBaseController = rospy.Publisher(base_controller_str.format('y'), Float64, queue_size=10)
        self.zBaseController = rospy.Publisher("/base/base_z_rotation_controller/command", Float64, queue_size=10)


    def sendEfforts(self, msg):
        # helper function to send Effort values to cim motors
        # msg [Joy] 
        # Code inspired by https://gm0.org/en/latest/docs/software/mecanum-drive.html

        L_JoyX = msg.axes[0]
        L_JoyY = msg.axes[1] # potentially multiply this value by 1.1 to counteract imperfect strafing!
        R_JoyX = -1*msg.axes[2] # flip rotation direction

        # publish to simulation
        self.xBaseController.publish(L_JoyX)
        self.yBaseController.publish(L_JoyY)
        self.zBaseController.publish(R_JoyX)

        demoninator = max(abs(L_JoyX) + abs(L_JoyY) + abs(R_JoyX), 1)

        # Front of robot
        # 1 3
        # 2 4
        # Rear of Robot

        cim1Effort = (R_JoyX + L_JoyX + L_JoyY) / demoninator * 100
        cim2Effort = (R_JoyX - L_JoyX + L_JoyY) / demoninator * 100
        cim3Effort = (R_JoyX - L_JoyX - L_JoyY) / demoninator * 100
        cim4Effort = (R_JoyX + L_JoyX - L_JoyY) / demoninator * 100

        efforts = effort_list()
        efforts.Efforts = [cim1Effort, cim2Effort, cim3Effort, cim4Effort]
        self.mecanum_efforts.publish(efforts)


    def updatePosition(self, msg):
        # TODO
        # Publish rough coordinates of x base y base and z rotation to update in sim!
        pass


if __name__=="__main__":
    m = MecanumNode(init_node=True)
    rospy.spin()