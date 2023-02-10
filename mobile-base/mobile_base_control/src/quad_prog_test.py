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
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

from visualization_msgs.msg import Marker

class holistic_control():

    def __init__(self, goalT):
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
        self.R = 0.1524 # wheel radius [m]
        self.W = 0.56848 # distance between the two wheels [m]
        self.lx = self.W/2 # represent the distance from the robot's center to the wheels projected on the x and y axis
        self.ly = 0.3617629 # represent the distance from the robot's center to the wheels projected on the x and y axis


        self.panda_arm = rtb.models.ETS.Panda() # instantiate arm from rtb


        # Listen to Joint States
        rospy.Subscriber(arm_joints_txt.format(arm = self.arm), JointState, self.armCallback, queue_size=10)
        rospy.Subscriber("/base/joint_states", JointState, self.baseCallback, queue_size=10)

        # Hookup Base Velocities
        base_controller_str = "/base/base_{0}_joint_controller/command"
        self.Vx_world = rospy.Publisher(base_controller_str.format('x'), Float64, queue_size=10)
        self.Vy_world = rospy.Publisher(base_controller_str.format('y'), Float64, queue_size=10)
        self.W_z_world = rospy.Publisher("/base/base_z_rotation_controller/command", Float64, queue_size=10)

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

        # Initialize Marker in RVIZ
        mark = Marker()
        mark.header.frame_id = "world"
        mark.header.stamp = rospy.Time.now()
        mark.ns = "panda"
        mark.id = 0 #arrow
        mark.type = 0
        # mark.action = Marker().ADD
        mark.pose.position.x = goalT[0,-1]
        mark.pose.position.y = goalT[1,-1]
        mark.pose.position.z = goalT[2,-1]
        print(goalT[:3,:3])
        r = R.from_matrix(goalT[:3,:3]) # rotation matrix
        (x, y, z, w) = r.as_quat()
        mark.pose.orientation.x = x
        mark.pose.orientation.y = y
        mark.pose.orientation.z = z
        mark.pose.orientation.w = w
        mark.color.a = 1
        mark.color.r = 1
        mark.color.b = 0
        mark.color.g = 0
        mark.scale.x= .1
        mark.scale.y= .1
        mark.scale.z= .1
        mark.lifetime.secs = 0 # FOREVER
        print(mark)
        markPub = rospy.Publisher("/visualization_marker", Marker, queue_size=2)
        rospy.sleep(0.5)
        markPub.publish(mark)
        

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

        spatial_error = 1 # init
        while spatial_error >= 0.005:

            ### GET world to end effector transformation ###
            try: 
                root_joint_name = "panda_link8"
                position, quaternion = self.tf.lookupTransform("world", root_joint_name, rospy.Time())      
                r = R.from_quat(quaternion)
                w2e_T = np.hstack((r.as_matrix(), np.asarray(position).reshape((3,1))))
                w2e_T = np.vstack((w2e_T, np.array(([0,0,0,1]))))   


                # Get current angle of base
                root_joint_name = "base_footprint"
                w2b_pose, w2b_orient = self.tf.lookupTransform("world", root_joint_name, rospy.Time()) 

            except Exception as e:
                print(e)
                return #just retry instantly
            
            # Grab Manipulator Jacobian in EE frame
            arm_joints = self.arm_joint_states
            base_joints = self.base_joint_states
            base_xy = base_joints[:2]

            q_h = np.hstack((np.array([0, 0]), arm_joints))
            self.panda_base_arm.q = q_h # force update

            # Test plot accuracy
            # print(f"q_h: {q_h}")
            # self.panda_base_arm.plot(q_h)
            # rospy.sleep(10000)

            
            mJacob = self.panda_base_arm.jacobe(q_h)
            # Make Equality Constraints
            Aeq = np.c_[mJacob, np.eye(6)]
            
            # fkine = self.panda_base_arm.fkine(q_h)
            # # print(f"fkine.A {fkine.A}")
            # # return
            v, _ = rtb.p_servo(w2e_T, goalT, 1.5)
            v[3:] *= 1.3 # rotate faster?
            
            beq = v.reshape((6,))

            ## The Inequality Constraints for joint limit avoidance ##
            Ain = np.zeros((self.arm_dof+2 + 6, self.arm_dof+2 + 6))
            bin = np.zeros(self.arm_dof+2 + 6)

            ps = 0.2

            pi = 0.9

            # Form the joint limit velocity damper
            qlim_h = self.panda_base_arm.qlim
            Ain[: self.arm_dof+2, : self.arm_dof+2], bin[: self.arm_dof+2] = arm_utilities.joint_velocity_damper(ps, pi, n=self.arm_dof+2, q=q_h, qlim=qlim_h)
            # Linear component of objective function: the manipulability Jacobian
            c = np.concatenate(
                (np.zeros(2), -self.panda_arm.jacobm(q=arm_joints).reshape((self.arm_dof)), np.zeros(6))
            )
            
            # Get base to face end-effector
            kε = 0.3
            b2e_T = self.panda_arm.fkine(q=arm_joints).A
            θε = np.arctan2(b2e_T[1, -1], b2e_T[0, -1])
            ε = kε * θε
            c[0] = -ε


            # The lower and upper bounds on the joint velocity and slack variable
            lb = -np.r_[np.array(([2,0.5])), np.asarray(self.qd_Limits), 10 * np.ones(6)]
            ub = np.r_[np.array(([2,0.5])), np.asarray(self.qd_Limits), 10 * np.ones(6)]

            errorT = np.linalg.inv(w2e_T) @ goalT
            # print(f"eTep: {errorT}")
            spatial_error = np.sum(np.abs(errorT[:3, -1]))
            # print(f"Spatial Error: {spatial_error}")
            Q = np.eye(self.arm_dof + 2 + 6 )
            k_a = 0.01
            Q[: self.arm_dof+2, : self.arm_dof+2] *= k_a # apply k_a to arm joints in Q
            Q[:2, :2] *= 1.0/spatial_error

            # Slack Q
            Q[self.arm_dof+2 :, self.arm_dof+2 :] = (1.0 / spatial_error) * np.eye(6)
            
            # print(f"Q: {Q}")
            # print(f"c: {c.shape}")
            # print(f"lb {lb.shape}")
            # print(f"ub: {ub.shape}")
            # print(f"Aeq: {Aeq.shape}")
            # print(f"Beq: {beq.shape}")
            qd = solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver='cvxopt')
            # qd = qd[: self.arm_dof+2]
            # print(f"qd: {qd}")


            # Send motion to arm joints
            arm_qd = [qd[2], qd[3], qd[4], qd[5], qd[6], qd[7], qd[8]]
            #self.sendArmVels_trajectories(vels)
            self.sendArmVels(arm_qd)

            # Send motion to base
            # mecanum_FK(self, w_fl, w_fr, w_rl, w_rr, r, lx, ly))
            
            # Transform Qd into sent joint commands
            # Turn Virtual Joints into motion
            qd_bθ = qd[0]
            qd_bδ = qd[1]
            w_l = (qd_bδ - self.W * qd_bθ)/self.R
            w_r = (qd_bθ * self.W)/self.R + w_l

            (Vx_base, Vy_base, W_z_base) = arm_utilities.mecanum_FK(w_l, w_r, w_l, w_r, self.R, self.lx, self.ly)

            euler = euler_from_quaternion(w2b_orient) # translate to euler
            curr_val = euler[2] # z rotation
            # print(f"Euler Angles: {curr_val}")

            if curr_val < 0:
                curr_val = curr_val + 2*np.pi # maps from 0-2pi now


            # Create 2D rotation matrix around local chassis Z axis (rotation)
            R_z = np.array(([np.cos(curr_val), -np.sin(curr_val), 0],
                            [np.sin(curr_val), np.cos(curr_val), 0],
                            [0, 0, 1]))

            # Rotate Robot XY velocities to World XY Velocities 
            worldV_XY = R_z@np.array(([Vx_base, Vy_base, 0]))

            self.Vx_world.publish(worldV_XY[0])
            self.Vy_world.publish(worldV_XY[1])
            self.W_z_world.publish(W_z_base)

            print(f"{W_z_base} {qd_bθ} {W_z_base-qd_bθ}")
        
        self.sendArmVels([0,0,0,0,0,0,0])
        self.Vx_world.publish(0)
        self.Vy_world.publish(0)
        self.W_z_world.publish(0)
        
        



if __name__ == "__main__":
    
    goalT = np.array(([-1, 0, 0, 2.3],
                      [0, 1, 0, 0],
                      [0, 0, -1, 1.45],
                      [0, 0, 0, 1]))

    holistic = holistic_control(goalT=goalT)
    # holistic.__init__(goalT)

    holistic.main_loop(goalT)
    # rospy.spin()
