#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray, Float64
import numpy as np
import matplotlib.pyplot as plt
import pickle

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

                    # Pickle Data to compare later
                    # with open('elevzxy_calpha_hard.pkl', 'wb') as handle:
                    #     pickle.dump([self.mListTime, self.mList, self.mbListTime, self.mbList], handle, protocol=pickle.HIGHEST_PROTOCOL)

                    plt.show()
                    break
        except KeyboardInterrupt:
            pass
        pass


    def analyze_pickles(self):

        file1= open("/home/edward/Classes/Thesis/elevz_hard.pkl", 'rb')    
        mListTime1, mList1, mbListTime1, mbList1 = pickle.load(file1)
        file1.close()
        
        file2= open("/home/edward/Classes/Thesis/elevxyz_corke2_hard.pkl", 'rb')      
        mListTime2, mList2, mbListTime2, mbList2 = pickle.load(file2)
        file2.close()
        
        file3= open("/home/edward/Classes/Thesis/elevzxy_noManip_hard.pkl", 'rb')      
        mListTime3, mList3, mbListTime3, mbList3 = pickle.load(file3)
        file3.close()
        
        file4= open("/home/edward/Classes/Thesis/elevzxy_calpha_hard.pkl", 'rb')      
        mListTime4, mList4, mbListTime4, mbList4 = pickle.load(file4)
        file4.close()
        
        # file5= open("/home/edward/Classes/Thesis/elevxyz_noManip_common.pkl", 'rb')      
        # mListTime5, mList5, mbListTime5, mbList5 = pickle.load(file5)
        # file5.close()

        figure, (ax1, ax2) = plt.subplots(2, 1)
        ax1.plot(mListTime1, mList1,  label="P:Z")
        ax1.plot(mListTime3, mList3,  label="No Maximization")
        ax1.plot(mListTime4, mList4,  label="C Alpha")
        # ax1.plot(mListTime5, mList5,  label="No Maximization")
        ax1.plot(mListTime2, mList2,  label="Corke")

        ax1.set_title("Manipulability with Levels of Elevator Integration")
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Yoshikawa Manipulability')

        figure.legend()
        ax1.grid()
        plt.show()

if __name__ == "__main__":
    listener = holistic_listener()
    listener.plotData()
    # listener.analyze_pickles()

   
