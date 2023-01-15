#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from math import pi
from base_package.srv import moveToElev


class BaseClient:
    def __init__(self, init_node=False) -> None:

        if init_node:
            rospy.init_node("base_client", anonymous=True)

        rospy.sleep(1)

    def move_elevator(self, height):
        # client side used to move the arm to a desired EE pose

        rospy.wait_for_service('/elevation_request')
        try:
            c = rospy.ServiceProxy('/elevation_request', moveToElev)
            success = c(height)
            bool = Bool()
            bool.data = True
            if success.data:
                return
            else:
                return print(str(success.data.data) + ", unable to find a solution.")

        except Exception as e:
            print("Elevator service called failed as: %s"%e)

    def home_elevator(self, msg):
        pass #TODO: fill in


if __name__ == "__main__":
    b = BaseClient(init_node=True)

    b.move_elevator(10000)
