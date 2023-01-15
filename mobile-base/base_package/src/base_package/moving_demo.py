#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import time

class MoveDemo(object):
    def __init__(self):
        rospy.init_node("random_move")
        self.rate = rospy.Rate(.5)

        self.base_controllers = []
        self.arm_controllers = []

        arm_name = rospy.get_param("arm/name")
        arm_namespace = "panda" if arm_name=="panda" else "my_gen3"

        arm_controller_str = "/" + arm_namespace + "/" + arm_name + "_{0}_joint_controller/command"
        base_controller_str = "/base/base_{0}_joint_controller/command"

        self.base_controllers.append(rospy.Publisher(base_controller_str.format('x'), Float64, queue_size=10))
        self.base_controllers.append(rospy.Publisher(base_controller_str.format('y'), Float64, queue_size=10))
        self.base_controllers.append(rospy.Publisher(base_controller_str.format('z'), Float64, queue_size=10))
        self.base_controllers.append(rospy.Publisher("/base/base_z_rotation_controller/command", Float64, queue_size=10))

        self.arm_controllers.append(rospy.Publisher(arm_controller_str.format('x'), Float64, queue_size=10))
        self.arm_controllers.append(rospy.Publisher(arm_controller_str.format('y'), Float64, queue_size=10))
        self.arm_controllers.append(rospy.Publisher(arm_controller_str.format('z'), Float64, queue_size=10))
        self.arm_controllers.append(rospy.Publisher("/" + arm_namespace + "/" + arm_name + "_z_rotation_controller/command", Float64, queue_size=10))


    def back_and_forth(self):
        value = 0
        stop = False
        self.base_controllers[0].publish(Float64(-10))
        while not rospy.is_shutdown():
            stop = not stop
            value += .1

            print("sending value of " + str(value if stop else 0))
            # self.base_controllers[2].publish(value if stop else 0)
            self.arm_controllers[2].publish(value if stop else 0)
            self.arm_controllers[1].publish(0)
            self.arm_controllers[0].publish(0)

            time.sleep(1)

if __name__ == '__main__':
    try:
        print("running main")
        d = MoveDemo()
        d.back_and_forth()
    except rospy.ROSInterruptException:
        print("base movement node error")
        
