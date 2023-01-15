#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np


## UTILIZE TO TURN PRISMATIC WORLD JOINTS INTO MECANUM MOTION ##
# Math based from https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html

class sim_mecanum_logic():
    def __init__(self) -> None:
        pass

    def mecanum_FK(self, w_fl, w_fr, w_rl, w_rr, r, lx, ly):
        """
        w_fl, w_fr, w_rl, w_rr -> wheel velocities
        Returns Vx, Vy, and W_z
        """
        a_ = np.array([1, 1, 1, 1],
                      [-1, 1, 1, -1],
                      [-1/(lx+ly), 1/(lx+ly), -1/(lx+ly), 1/(lx+ly)])
        b_ = np.array([w_fl], [w_fr], [w_rl], [w_rr])

        FK = r/4@a_@b_

        return FK

    def mecanum_IK(self, v_x, v_y, w_z, r, lx, ly):
        """
        """
        a_ = np.array([1, -1, -(lx+ly)],
                      [1, 1, (lx+ly)],
                      [1, 1, -(lx+ly)],
                      [1, -1, (lx+ly)])
        b_ = np.array([v_x], [v_y], [w_z])

        IK = 1/r@a_@b_

        return IK



if __name__ == "__main__":
    sim_mecanum = sim_mecanum_logic()

