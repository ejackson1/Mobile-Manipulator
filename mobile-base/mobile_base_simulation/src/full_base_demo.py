#!/usr/bin/env python3

from base_package.base_sim_control import BaseControl
from abstract_arm.moveItClient import MoveitArmClient
import time
import geometry_msgs.msg
import rospy
from math import pi

class FullBaseDemo:
    def __init__(self):
        self.base_controller = BaseControl()
        self.arm_controller = MoveitArmClient()

        print("initializing base and arm controller")
        time.sleep(3)


if __name__ == "__main__":
    f = FullBaseDemo()

    print("moving base")
    f.base_controller.base_z_rotation_pos.publish(1.0)
    f.base_controller.base_x_vel.publish(-2.0)
    f.base_controller.base_y_vel.publish(-1.0)
    
    time.sleep(2)

    f.base_controller.base_z_rotation_pos.publish(0.0)
    f.base_controller.base_x_vel.publish(0.0)
    f.base_controller.base_y_vel.publish(0.0)

    time.sleep(1)

    print("raising elevator")
    f.base_controller.insert_z_pos.publish(.3)

    print("moving arm to a position")
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.5
    pose_goal.position.y = -0.5
    pose_goal.position.z = 0.6
    pose_goal.orientation.x = pi
    f.arm_controller.move_arm(pose_goal)

    time.sleep(2)

    print("raising elevator")
    f.base_controller.insert_z_pos.publish(-.2)

    time.sleep(2)

    print("moving base back")
    f.base_controller.base_x_vel.publish(2.0)
    f.base_controller.base_y_vel.publish(1.0)

    time.sleep(2)

    f.base_controller.base_x_vel.publish(0.0)
    f.base_controller.base_y_vel.publish(0.0)