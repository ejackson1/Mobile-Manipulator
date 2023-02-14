#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray, Float64
import numpy as np
import matplotlib.pyplot as plt

"""
Utilized only to collect data and evaluate the performance of the holistic controller performance
"""

class holistic_listener():
    
    def __init__(self):
        # Init node
        rospy.init_node("holistic_listener")
        self.arm = rospy.get_param("/arm/name")
        arm_vel_txt = "{arm}/gazebo_ros_control/command"
        self.velList = []
        self.velListTime = []

        self.mList = []
        self.mListTime = []

        ## Subscribers
        rospy.Subscriber(arm_vel_txt.format(arm=self.arm), Float64MultiArray, self.armVelCallback, queue_size=10)
        rospy.Subscriber("{arm}/manipulability".format(arm=self.arm), Float64, self.armManipCallback, queue_size=10)


    def armVelCallback(self, msg):
        """
        Callback function recieving the joint velocities of the arm. Saves data into a plot
        """
        self.qd = msg.data
        self.qd_time = rospy.get_rostime()
        self.qd_time = self.qd_time.to_sec() # + self.qd_time.to_nsec()  # rewrite into seconds

        # Append to List for later plotting
        self.velList.append(self.qd)
        self.velListTime.append(self.qd_time)


    def armManipCallback(self, msg):
        """
        Callback function recieving the manipulability measure of the arm. Saves data into a plot
        """

        self.m = msg.data
        self.mTime = rospy.get_rostime()
        self.mTime = self.mTime.to_sec()
        # print(f"Manip {self.m}")
        # print(f"time {self.mTime}")
        # Append to List for later plotting
        self.mList.append(self.m)
        self.mListTime.append(self.mTime)


    def plotData(self):
        while 1:
            goalReached = rospy.get_param('/goal_reached')
            if goalReached == True:
                figure, (ax1, ax2) = plt.subplots(2, 1)
                # axis[0,0].plot(self.velListTime, self.velList)
                # axis[0,0].set_title("Velocity Over Time")
                ax1.plot(self.mListTime, self.mList)
                ax1.set_title('Manipulability OverTime')
                ax1.set_ylabel('Yoshikawa Index')
                ax1.set_xlabel('Time (s)')



                plt.show()
        pass


if __name__ == "__main__":
    listener = holistic_listener()
    listener.plotData()

   
