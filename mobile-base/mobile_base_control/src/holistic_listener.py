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
        self.mbList = []
        self.mbListTime = []
        self.mModList = []
        self.mModListTime = []

        ## Subscribers
        rospy.Subscriber(arm_vel_txt.format(arm=self.arm), Float64MultiArray, self.armVelCallback, queue_size=10)
        rospy.Subscriber("{arm}/manipulability".format(arm=self.arm), Float64, self.armManipCallback, queue_size=10)
        rospy.Subscriber("{arm}_base/manipulability".format(arm=self.arm), Float64, self.armBaseManipCallback, queue_size=10)
        rospy.Subscriber("{arm}/modJacManipulabilitity".format(arm=self.arm), Float64, self.armModManipCallback, queue_size=10)


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

    def armModManipCallback(self, msg):
        """
        Callback function recieving the manipulability measure of the arm. Saves data into a plot
        """

        self.mMod = msg.data
        self.mModTime = rospy.get_rostime()
        self.mModTime = self.mModTime.to_sec()
        # print(f"Manip {self.m}")
        # print(f"time {self.mTime}")
        # Append to List for later plotting
        self.mModList.append(self.mMod)
        self.mModListTime.append(self.mModTime)
    
    def armBaseManipCallback(self, msg):
        """
        Callback function recieving the manipulability measure of the arm & base. Saves data into a plot
        """

        self.mb = msg.data
        self.mbTime = rospy.get_rostime()
        self.mbTime = self.mbTime.to_sec()
        # print(f"Manip {self.m}")
        # print(f"time {self.mTime}")
        # Append to List for later plotting
        self.mbList.append(self.mb)
        self.mbListTime.append(self.mbTime)


    def plotData(self):
        k = "1"
        try:
            while 1:
                goalReached = rospy.get_param('/goal_reached')
                if goalReached == True:
                    figure, (ax1, ax2) = plt.subplots(2, 1)
                    # figure(figsize=((6,10)))

                    figure.set_figwidth(8)
                    figure.set_figheight(18)
                    
                    # Reinitalize times so they're not sim times
                    self.mListTime = np.asarray(self.mListTime)
                    self.mbListTime = np.asarray(self.mbListTime)
                    # self.mModListTime = np.asarray(self.mModListTime)

                    self.mListTime -= self.mListTime[0]
                    self.mbListTime -= self.mbListTime[0]
                    # self.mModListTime -= self.mModListTime[0]

                    ax1.plot(self.mListTime, self.mList)
                    ax1.set_title('Manipulability (Arm), k={k}'.format(k=k))
                    ax1.set_ylabel('Yoshikawa Index')
                    ax1.set_xlabel('Time (s)')

                    ax2.plot(self.mbListTime, self.mbList)
                    ax2.set_title('Manipulability (Base+Arm), k={k}'.format(k=k))
                    ax2.set_ylabel('Yoshikawa Index')
                    ax2.set_xlabel('Time (s)')

                    # ax3.plot(self.mModListTime, self.mModList)
                    # ax3.set_title('Modified Jacobian Manipulability (Base+Arm), k={k}'.format(k=k))
                    # ax3.set_ylabel('Yoshikawa Index')
                    # ax3.set_xlabel('Time (s)')


                    plt.show()
        except KeyboardInterrupt:
            pass
        pass


if __name__ == "__main__":
    listener = holistic_listener()
    listener.plotData()

   
