#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64
from tf import TransformListener, TransformBroadcaster
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import LinkState

"""
Designed to keep the simulated revolute joints superimposed ontop of eachother instead
of a serial chain. These joints provide rotation about the x and y axis on the base of
the manipulator arm. They will be equivalent to "panda_link0" but with a motor attached the 
x and y axis'. 

ROS joints are serial which means the revolute X joint is stationed before the revolute Y joint, 
instead of being the "same" joint. If motor Y rotates, the coordinate system for joint X
will not rotate. Instead, we want joint rotation of X or Y to affect both coordinate systems.
"""


class revolute_follower():


    def __init__(self) -> None:
        rospy.init_node("revolute_follower")
        self.arm = rospy.get_param("/arm/name")

        self.tf = TransformListener()   
        self.br = TransformBroadcaster()

        self.Vz_world = rospy.Publisher("{arm}/panda_z_joint_controller/command".format(arm=self.arm), Float64, queue_size=10)

        self.gazeboLinkState = rospy.Publisher("/gazebo/set_link_state", LinkState, queue_size=10)

        rospy.sleep(0.5)     
        rospy.loginfo("Initialized revolute follower node")


    def adjustRevolute(self):

        while not rospy.is_shutdown(): # while ROS is running

            try:
                arm_x_link = "arm_x_link"
                arm_y_link = "arm_y_link"
                arm_z_link = "arm_z_link"
                panda_link0 = "panda_link0"
                panda_link1 = "panda_link1"
                panda_link2 = "panda_link1"
                panda_link3 = "panda_link1"
                panda_link4 = "panda_link1"
                panda_link5 = "panda_link1"
                panda_link6 = "panda_link6"
                panda_link7 = "panda_link7"
                panda_link7 = "panda_link7"
                panda_link7 = "panda_link7"

                x_joint_name = "arm_rx_link"
                y_joint_name = "arm_ry_link"
                parent_joint_name = "arm_x_link"

                positionX, quaternionX = self.tf.lookupTransform("world", x_joint_name, rospy.Time())
                positionY, quaternionY = self.tf.lookupTransform("world", y_joint_name, rospy.Time())
                print(f"PositionX: {tuple(positionX)}")
                print(f"PositionY: {positionY}")
                print(f"QuaternionX: {quaternionX}")
                print(f"QuaternionY: {quaternionY}")
                print(f"QuaternionY: {quaternionY[3]}")

                if not np.isclose(np.sum(np.asarray(quaternionX)) - np.sum(np.asarray(quaternionY)), 0, rtol=0.00005):
                    # send the transform 
                    print("sending")
                    # self.br.sendTransform((1,0,0), (0,0,0,1), rospy.Time.now(), "base_footprint", "world")
                    # self.br.sendTransform(positionY, quaternionY, rospy.Time.now(), y_joint_name, parent_joint_name)
                    linkUpdate = LinkState()
                    linkUpdate.link_name = y_joint_name
                    linkUpdate.pose.position.x = positionY[0]
                    linkUpdate.pose.position.y = positionY[1]
                    linkUpdate.pose.position.z = 0.9
                    linkUpdate.pose.orientation.x = quaternionX[0]
                    linkUpdate.pose.orientation.y = quaternionX[1]
                    linkUpdate.pose.orientation.z = quaternionX[2]
                    linkUpdate.pose.orientation.w = quaternionX[3]
                    linkUpdate.reference_frame = "world"
                    self.gazeboLinkState.publish(linkUpdate)



            except Exception as e:
                print(e)
                return # retry


if __name__ == "__main__":
    revFollower = revolute_follower()
    revFollower.adjustRevolute()
