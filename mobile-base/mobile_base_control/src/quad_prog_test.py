#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import numpy as np
from qpsolvers import solve_qp
from arm_utilities import arm_utilities
from panda_description import Panda_Modified as panda
import roboticstoolbox as rtb
from tf import TransformListener
from scipy.spatial.transform import Rotation as R
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray, MultiArrayLayout

class holistic_control():

    def __init__(self):
        # Initialize Node
        rospy.init_node("holistic_control")
        self.arm = rospy.get_param("/arm/name")
        self.arm_dof = rospy.get_param("/arm/dof")
        self.qd_Limits = []
        for i in range(self.arm_dof):
            self.qd_Limits.append(rospy.get_param("/robot_description_planning/joint_limits/panda_joint{i}/max_velocity".format(i=i+1)))
        
        arm_joints_txt = "{arm}/joint_states"
        arm_trajectory_txt = "{arm}/gazebo_ros_control/command"
        self.panda_base_arm = panda()
        print(f"self.panda_base_arm: {self.panda_base_arm}")
        


        self.panda_arm = rtb.models.ETS.Panda() # instantiate arm from rtb


        # Listen to Joint States
        rospy.Subscriber(arm_joints_txt.format(arm = self.arm), JointState, self.armCallback, queue_size=10)
        rospy.Subscriber("/base/joint_states", JointState, self.baseCallback, queue_size=10)

        # Hookup Base Velocities


        # Publishers
        # self.traj_client = rospy.Publisher(arm_trajectory_txt.format(arm=self.arm), JointTrajectory, queue_size=10)

        self.armQdPublisher = rospy.Publisher(arm_trajectory_txt.format(arm=self.arm), Float64MultiArray, queue_size=10)


        self.arm_joint_names = [f"{self.arm}_joint1",
                            f"{self.arm}_joint2",
                            f"{self.arm}_joint3",
                            f"{self.arm}_joint4",
                            f"{self.arm}_joint5",
                            f"{self.arm}_joint6",
                            f"{self.arm}_joint7"]
        

        self.tf = TransformListener()

        rospy.sleep(0.5)
        
    def armCallback(self, msg):
        # print(f"msg: {msg}")
        self.arm_joint_states = np.array(([msg.position[2],
                                           msg.position[3],
                                           msg.position[4],
                                           msg.position[5],
                                           msg.position[6],
                                           msg.position[7],
                                           msg.position[8],
                                          ]))
        return
    
    def baseCallback(self, msg):
        self.base_joint_states = np.array(([msg.position[0],    # x
                                            msg.position[1],    # y
                                            msg.position[2],    # z
                                            msg.position[3],])) # z_rot
        
        return

    def sendArmVels_Trajectories(self, vel):
        """
        Sends joint velocities to the robot arm
        Vel [list] - List of velocities [q1, q2, q3, ..., qn]
        """ # not functional anymore
        print(f"vels: {vel}")
        print(f"length of vel: {len(vel)}")
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = self.arm_joint_names
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = list(self.arm_joint_names) 
        arm_trajectory.points[0].velocities = vel
        arm_trajectory.points[0].accelerations = [0,0,0,0,0,0,0] 
        arm_trajectory.points[0].time_from_start = rospy.Duration(0.01)
        self.traj_client.publish(arm_trajectory)
        
        pass    
    

    def sendArmVels(self, vel):
        """
        Sends joint velocities to the robot arm
        Vel [list]
        """

        arm_qd = Float64MultiArray()
        arm_qd.data = vel

        arm_qd.layout = MultiArrayLayout()

        self.armQdPublisher.publish(arm_qd)

        


    def main_loop(self, goalT):




        ### GET world to end effector transformation ###
        try: 
            root_joint_name = "panda_link8"
            position, quaternion = self.tf.lookupTransform("world", root_joint_name, rospy.Time())      
            r = R.from_quat(quaternion)
            w2e_T = np.hstack((r.as_matrix(), np.asarray(position).reshape((3,1))))
            w2e_T = np.vstack((w2e_T, np.array(([0,0,0,1]))))   
            print(f"w2e_T {w2e_T}")
        except Exception as e:
            print(e)
            return #just retry instantly
        
        print(f"w2e_T: {w2e_T}")
        # Grab Manipulator Jacobian in EE frame
        # mJacob = arm_utilities.mJacobian(self.S, self.arm_dof, self.arm_joint_states)
        arm_joints = self.arm_joint_states
        base_joints = self.base_joint_states
        base_xy = base_joints[:2]

        q_h = np.hstack((np.array([0, 0]), arm_joints))
        self.panda_base_arm.q = q_h # force it to update
        # self.panda_arm_base.plot(q_h)

        # Test plot accuracy
        print(f"q_h: {q_h}")
        # self.panda_base_arm.plot(q_h)
        # rospy.sleep(10000)

        
        mJacob = self.panda_base_arm.jacobe(q_h)
        print(f"mJacob: {mJacob}")
        # Make Equality Constraints
        Aeq = np.c_[mJacob, np.eye(6)]
        
        fkine = self.panda_base_arm.fkine(q_h)
        print(f"fkine.A {fkine.A}")
        v, _ = rtb.p_servo(fkine.A, goalT, 1.5)
        v[3:] *= 1.3 # rotate faster?
        
        beq = v.reshape((6,))

        print(f"beq: {beq}")
        ## The Inequality Constraints for joint limit avoidance ##
        Ain = np.zeros((self.arm_dof+2 + 6, self.arm_dof+2 + 6))
        bin = np.zeros(self.arm_dof+2 + 6)

        # The minimum angle (in radians) in which the joint is allowed to approach
        # to its limit
        ps = 0.2

        # The influence angle (in radians) in which the velocity damper
        # becomes active
        pi = 0.9

        # Form the joint limit velocity damper
        print(f"armjoints: {arm_joints}")
        
        
        # print(f"qlim: {self.panda_arm.qlim}")
        print(f"q_h: {q_h}")
        qlim_h = self.panda_base_arm.qlim
        print(f"qlim_h {qlim_h}")
        Ain[: self.arm_dof+2, : self.arm_dof+2], bin[: self.arm_dof+2] = arm_utilities.joint_velocity_damper(ps, pi, n=self.arm_dof+2, q=q_h, qlim=qlim_h)
        print(f"Ain: {Ain.shape}\nAin {Ain}")
        print(f"bin: {bin.shape}\nBin {bin}")
        # Form into c
        # c = np.concatenate(
        #     (np.zeros(2), mJacob.reshape(self.arm_dof - 2), np.zeros(6))
        # )
        
        # Linear component of objective function: the manipulability Jacobian
    
        # print(f"jacobm: {-self.panda_arm.jacobm(start=self.panda_arm.links[4]).reshape((self.arm_dof))}")
        # print(f"jacobm before reshape: {-self.panda_arm.jacobm(start=self.panda_arm.links[4])}")
        c = np.concatenate(
            (np.zeros(2), -self.panda_base_arm.jacobm(start=self.panda_base_arm.links[4]).reshape((self.arm_dof)), np.zeros(6))
        )
        
        # Get base to face end-effector
        kε = 0.5
        # b2e_T = arm_utilities.fkine(self.S, self.M, self.arm_dof, self.arm_joint_states)
        b2e_T = self.panda_arm.fkine(q=arm_joints).A
        print(f"b2e_t {b2e_T}")
        θε = np.arctan2(b2e_T[1, -1], b2e_T[0, -1])
        ε = kε * θε
        c[0] = -ε


        # The lower and upper bounds on the joint velocity and slack variable
        lb = -np.r_[np.array(([4,4])), np.asarray(self.qd_Limits), 10 * np.ones(6)]
        # print(f"lb: {lb}")
        ub = np.r_[np.array(([4,4])), np.asarray(self.qd_Limits), 10 * np.ones(6)]

        errorT = np.linalg.inv(w2e_T) @ goalT
        print(f"eTep: {errorT}")
        spatial_error = np.sum(np.abs(errorT[:3, -1]))
        print(f"Spatial Error: {spatial_error}")
        Q = np.eye(self.arm_dof + 2 + 6 )
        k_a = 0.01
        Q[: self.arm_dof+2, : self.arm_dof+2] *= k_a # apply k_a to arm joints in Q
        Q[:2, :2] *= 1.0/spatial_error

        # Slack Q
        Q[self.arm_dof+2 :, self.arm_dof+2 :] = (1.0 / spatial_error) * np.eye(6)
        
        print(f"Q: {Q}")
        print(f"c: {c.shape}")
        print(f"lb {lb.shape}")
        print(f"ub: {ub.shape}")
        print(f"Aeq: {Aeq.shape}")
        print(f"Beq: {beq.shape}")
        qd = solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver='cvxopt')
        # qd = qd[: self.arm_dof+2]
        print(f"qd: {qd}")


        # Transform Qd into sent joint commands
        # Turn Virtual Joints into motion
        # qd_bθ = qd[0]
        # qd_bδ = qd[1]
        # w_l = (qd_bδ - self.W * qd_bθ)/self.R
        # w_r = (qd_bθ * self.W)/self.R + w_l

        ## TODO
        # send w_r and w_r to simulated_mecanum_logic mecanum_FK(w_fl, w_fr, w_rl, w_rr, r, lx, ly)

        # Send motion to arm joints
        vels = [qd[2], qd[3], qd[4], qd[5], qd[6], qd[7], qd[8]]
        #self.sendArmVels_trajectories(vels)
        self.sendArmVels(vels)



if __name__ == "__main__":
    holistic = holistic_control()
    goalT = np.array(([-1, 0, 0, 5],
                      [0, 1, 0, 0],
                      [0, 0, -1, 1.2],
                      [0, 0, 0, 1]))
    holistic.main_loop(goalT)
    # rospy.spin()
