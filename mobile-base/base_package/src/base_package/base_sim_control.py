#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import time

class BaseControl(object):
    def __init__(self):
        arm_name = rospy.get_param("arm/name")
        arm_namespace = "panda" if arm_name=="panda" else "my_gen3"

        insert_controller_str = "/" + arm_namespace + "/" + arm_name + "_{0}_joint_controller/command"
        base_controller_str = "/base/base_{0}_joint_controller/command"

        self.base_x_vel = rospy.Publisher(base_controller_str.format('x'), Float64, queue_size=10)
        self.base_y_vel = rospy.Publisher(base_controller_str.format('y'), Float64, queue_size=10)
        self.base_z_vel = rospy.Publisher(base_controller_str.format('z'), Float64, queue_size=10)
        self.base_z_rotation_pos = rospy.Publisher("/base/base_z_rotation_controller/command", Float64, queue_size=10)

        self.insert_x_pos = rospy.Publisher(insert_controller_str.format('x'), Float64, queue_size=10)
        self.insert_y_pos = rospy.Publisher(insert_controller_str.format('y'), Float64, queue_size=10)
        self.insert_z_pos = rospy.Publisher(insert_controller_str.format('z'), Float64, queue_size=10)
        self.insert_z_rotation_pos = rospy.Publisher("/" + arm_namespace + "/" + arm_name + "_z_rotation_controller/command", Float64, queue_size=10)

        rospy.init_node("base_control_node", anonymous=True)


if __name__ == '__main__':
    b = BaseControl()

    print("moving base forward")
    b.base_x_vel.publish(1.0)
