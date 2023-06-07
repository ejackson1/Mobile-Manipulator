#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayLayout


class JointVelocityController():

    def __init__(self) -> None:
        
        rospy.init_node("velocity_control")
        self.arm = rospy.get_param("/arm/name")
        arm_joints_txt = "{arm}/joint_states"
        arm_trajectory_txt = "{arm}/gazebo_ros_control/command"
        rospy.Subscriber(arm_joints_txt.format(arm = self.arm), JointState, self.armCallback, queue_size=10)

        self.armQdPublisher = rospy.Publisher(arm_trajectory_txt.format(arm=self.arm), Float64MultiArray, queue_size=10)
        
        rospy.sleep(0.1)
    
    def armCallback(self, msg):
        self.arm_joint_states = np.array(([msg.position[2],
                                           msg.position[3],
                                           msg.position[4],
                                           msg.position[5],
                                           msg.position[6],
                                           msg.position[7],
                                           msg.position[8],
                                          ]))
        return

    def sendArmVels(self, vel):
        """
        Sends joint velocities to the robot arm
        Vel [list]
        """

        arm_qd = Float64MultiArray()
        arm_qd.data = vel

        arm_qd.layout = MultiArrayLayout()

        self.armQdPublisher.publish(arm_qd)

    def mainFeedBackLoop(self, jointAngles_d):
        """
        Moves the Panda Arm to the desired Joint angles using a velocity based controller
        """
        K_p = 1 # proportional gain

        e = 1
        while np.linalg.norm(e) > 0.005:
            # Move joint angles into 0,2pi domain
            # jointAngles = np.where(self.arm_joint_states < 0, self.arm_joint_states + 2*np.pi, self.arm_joint_states)
            jointAngles = self.arm_joint_states
            jointAngles = np.reshape(jointAngles, (7,1))
            print(f"Joint Angles: {jointAngles}")
            print(f"Joint DesiredL {jointAngles_d}")

            # Find the error
            e = jointAngles_d - jointAngles
            e = np.reshape(e, (7,1))
            # Multiply error with proportional gain term
            q_dot = K_p * e

            # Send Joint Velocities to arm
            arm_qd = [float(q_dot[0]),
                      float(q_dot[1]),
                      float(q_dot[2]),
                      float(q_dot[3]),
                      float(q_dot[4]),
                      float(q_dot[5]),
                      float(q_dot[6])]
            print(arm_qd)
            self.sendArmVels(arm_qd)


        self.sendArmVels([0,0,0,0,0,0,0]) # freeze arm
        return

if __name__ == "__main__":
    velController = JointVelocityController()
    jointAngles_d = np.array(([0],
                              [-0.3],
                              [0],
                              [-2.2],
                              [0],
                              [2],
                              [0.78539816]))

    # Remap angles
    # jointAngles_d = np.where(jointAngles_d < 0, jointAngles_d + 2*np.pi, jointAngles_d)

    velController.mainFeedBackLoop(jointAngles_d)
