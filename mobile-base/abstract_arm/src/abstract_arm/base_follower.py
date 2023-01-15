#!/usr/bin/env python3
from threading import current_thread
import rospy
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from tf import TransformListener
from tf.transformations import euler_from_quaternion


class Follower(object):
    def __init__(self):
        rospy.init_node("base_follower")
        self.tf = TransformListener()

        self.arm_controllers = []
        self.prev_value = [0,0,0]

        arm_name = rospy.get_param("arm/name")
        print("arm: " + arm_name)
        arm_namespace = "panda" if arm_name=="panda" else "my_gen3"
        self.root_joint_name = "base_footprint" # "panda_link0" if arm_name=="panda" else "base_footprint" # idk why the panda needs the kortex base link but whatever

        arm_controller_str = "/" + arm_namespace + "/" + arm_name + "_{0}_joint_controller/command"

        self.arm_controllers.append(rospy.Publisher(arm_controller_str.format('x'), Float64, queue_size=10))
        self.arm_controllers.append(rospy.Publisher(arm_controller_str.format('y'), Float64, queue_size=10))
        self.arm_controllers.append(rospy.Publisher(arm_controller_str.format('z'), Float64, queue_size=10))
        self.arm_controllers.append(rospy.Publisher("/" + arm_namespace + "/" + arm_name + "_z_rotation_controller/command", Float64, queue_size=10))

        rospy.Subscriber("tf/", TFMessage, self.callback)

        rospy.spin()

    def callback(self, data):
        try:
            # get position and quaternion between world and base
            position, quaternion = self.tf.lookupTransform("world", self.root_joint_name, rospy.Time())

            if position != [0.0,0.0,0.0]:    
                # print(position)

                euler = euler_from_quaternion(quaternion) # translate to euler
                curr_val = (position[0], position[1], euler[2]) # x, y pos and z rotation
                # print(curr_val)

                # calc all differences between prev and curr value
                # differences = list(map(lambda x,y: abs(x - y), curr_val, self.prev_value))
                # print(differences)

                # if differences are within .2
                # if all(d < .2 for d in differences):
                    # publish to arm

                self.arm_controllers[0].publish(curr_val[0])
                self.arm_controllers[1].publish(curr_val[1])
                self.arm_controllers[3].publish(curr_val[2])

                # self.arm_controllers[2].publish(1)
                
                # self.prev_value = curr_val
        except Exception as e:
            print(e)
            pass
        

if __name__ == '__main__':
    try:
        d = Follower()
    except rospy.ROSInterruptException:
        print("Error following base")
        
