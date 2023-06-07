#!/usr/bin/env python3

# Math Imports
import numpy as np
from qpsolvers import solve_qp
from scipy.spatial.transform import Rotation as R

# Misc Imports
import copy
import pickle

# ROS Imports
import rospy
import rospkg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, Float64, Bool
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from tf import TransformListener
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import SpawnModel

# Robotic Toolbox imports 
import roboticstoolbox as rtb

# Stack Packages Imports
from arm_utilities import arm_utilities
from panda_description import Panda_Modified as panda
from holistic_listener import holistic_listener as listener



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
        arm_vel_txt = "{arm}/gazebo_ros_control/command"
        self.panda_base_arm = panda()
        print(f"self.panda_base_arm: {self.panda_base_arm}")
        self.R = 0.1524 # wheel radius [m]
        self.W = 0.56848 # distance between the two wheels [m]
        self.lx = self.W/2 # represent the distance from the robot's center to the wheels projected on the x and y axis
        self.ly = 0.3617629 # represent the distance from the robot's center to the wheels projected on the x and y axis
        self.w2l0_init = 1.0

        self.spatialErrorList = []
        self.sETimeList = []

        self.spatialErrorList.append(0)
        self.sETimeList.append(0)

        self.panda_arm = rtb.models.ETS.Panda() # instantiate arm from rtb


        # Listen to Joint States
        rospy.Subscriber(arm_joints_txt.format(arm = self.arm), JointState, self.armCallback, queue_size=10)
        rospy.Subscriber("/base/joint_states", JointState, self.baseCallback, queue_size=10)

        ### Publishers ###
        ## Velocity Publishers ##
        # Arm Velocity Publisher
        self.armQdPublisher = rospy.Publisher(arm_vel_txt.format(arm=self.arm), Float64MultiArray, queue_size=10)

        # Base Velocity Publishers
        base_controller_str = "/base/base_{0}_joint_controller/command"
        self.Vx_world = rospy.Publisher(base_controller_str.format('x'), Float64, queue_size=10)
        self.Vy_world = rospy.Publisher(base_controller_str.format('y'), Float64, queue_size=10)
        self.W_z_world = rospy.Publisher("/base/base_z_rotation_controller/command", Float64, queue_size=10)
        
        # Elevator Velocity Publishers
        self.Vz_world = rospy.Publisher("{arm}/panda_z_joint_controller/command".format(arm=self.arm), Float64, queue_size=10)
        self.rx_elev =  rospy.Publisher("{arm}/panda_rx_joint_controller/command".format(arm=self.arm), Float64, queue_size=10)
        self.ry_elev =  rospy.Publisher("{arm}/panda_ry_joint_controller/command".format(arm=self.arm), Float64, queue_size=10)


        # RVIZ Publisher
        markPub = rospy.Publisher("/RVIZ_goal", Marker, queue_size=2)

        # Manipulability Metric Publisher
        self.manipPub = rospy.Publisher("{arm}/manipulability".format(arm=self.arm), Float64, queue_size=1)
        self.manipBasePub = rospy.Publisher("{arm}_base/manipulability".format(arm=self.arm), Float64, queue_size=1)
        self.modJacPub = rospy.Publisher("{arm}/modJacManipulabilitity".format(arm=self.arm), Float64, queue_size=1)

        # Set Goal Reached Param
        self.reached_goal = Bool()
        self.reached_goal.data = False
        rospy.set_param('/goal_reached', self.reached_goal.data)

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
        mark.action = Marker().ADD
        mark.pose.position.x = goalT[0,-1]
        mark.pose.position.y = goalT[1,-1]
        mark.pose.position.z = goalT[2,-1]
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
        
        rospy.sleep(0.5)
        markPub.publish(mark)

        # Spawn object goal into Gazebo for debugging
        # STL from https://www.thingiverse.com/thing:1983449
        # TODO Make this a condition
        spawm_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        goalPose = Pose()
        goalPose.position.x = goalT[0,-1]
        goalPose.position.y = goalT[1,-1]
        goalPose.position.z = goalT[2,-1]
        goalPose.orientation.x = x
        goalPose.orientation.y = y
        goalPose.orientation.z = z
        goalPose.orientation.w = w
        rospack = rospkg.RosPack()
        spawm_model_client(
            model_name='goal_CS',
            model_xml=open(rospack.get_path('mobile_base_control') + '/urdf/right_hand_system_assembled.xml').read(),
            robot_namespace='/panda',
            initial_pose=goalPose,
            reference_frame='world'
        )

        

        self.tf = TransformListener()
        

        rospy.sleep(0.5)
        
    def armCallback(self, msg):
        self.arm_joint_states = np.array(([msg.position[9],
                                           msg.position[10], 
                                           msg.position[2],
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


    def sendArmVels(self, vel):
        """
        Sends joint velocities to the robot arm
        Vel [list]
        """

        arm_qd = Float64MultiArray()
        arm_qd.data = vel

        arm_qd.layout = MultiArrayLayout()

        self.armQdPublisher.publish(arm_qd)

    def yoshikawa(self, J):
        m2 = np.linalg.det(J @ J.T)
        return np.sqrt(abs(m2))        

    def main_loop(self, goalT):
        
        base_dofs = 2
        elev_dof_z = 1
        elev_dof_rxy = 2

        spatial_error = 1 # init
        while spatial_error >= 0.01:

            ### GET world to end effector transformation ###
            try: 
                root_joint_name = "panda_hand"
                position, quaternion = self.tf.lookupTransform("world", root_joint_name, rospy.Time())      
                if position == [0.0,0.0,0.0]: return
                r = R.from_quat(quaternion)
                w2e_T = np.hstack((r.as_matrix(), np.asarray(position).reshape((3,1))))
                w2e_T = np.vstack((w2e_T, np.array(([0,0,0,1]))))   


                # Get current angle of base
                root_joint_name = "base_footprint"
                w2b_pose, w2b_orient = self.tf.lookupTransform("world", root_joint_name, rospy.Time()) 
                
                # Find Elevator Position
                root_joint_name = "panda_link0"
                w2l0_pose, w2l0_orient = self.tf.lookupTransform("world", root_joint_name, rospy.Time()) 


            except Exception as e:
                print(e)
                return #just retry instantly

            # Grab Manipulator Jacobian in EE frame
            arm_joints = self.arm_joint_states
            base_joints = self.base_joint_states
            base_xy = base_joints[:2]
            q_bε = w2l0_pose[-1] - self.w2l0_init
            # print(f"w2lo_pose {q_bε}")
            armJointsElev = np.hstack((np.array([0,0,q_bε]), arm_joints))

            # q_h = np.hstack((np.array([0, 0, 0]), arm_joints))
            self.panda_base_arm.q = armJointsElev # force update
            # self.panda_base_arm.q = q_h # force update

            # Test plot accuracy
            # print(f"q_h: {armJointsElev}")
            # self.panda_base_arm.plot(armJointsElev, block=True)
            # rospy.sleep(10000)


            errorT = np.linalg.inv(w2e_T) @ goalT
            errorZdim = errorT[2,-1]
            spatial_error = np.sum(np.abs(errorT[:3, -1]))

            # Append spatial error to list & record the time
            self.spatialErrorList.append(spatial_error)
            self.sETimeList.append(rospy.get_rostime().to_sec())
            
            v, _ = rtb.p_servo(w2e_T, goalT, 0.75)
            v[3:] *= 1.3 # rotate faster

            eJacob = self.panda_base_arm.jacobe(q=armJointsElev)

            # Make Equality Constraints
            Aeq = np.c_[eJacob, np.eye(6)]  
            beq = v.reshape((6,))

            ## The Inequality Constraints for joint limit avoidance ##
            Ain = np.zeros((self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs + 6, self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs + 6))
            bin = np.zeros(self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs + 6)

            
            # Form the joint limit velocity damper
            # We treat the elevator joints differently than the arm joints
            # Elevator limits
            ps_elev = 0.05 # m or radians as a hardstop end point from limit
            pi_elev = 0.25 # m or radians to start the damper from limit

            # Limits of entire system from panda_description
            qlim_h = self.panda_base_arm.qlim
            # Ain[: elev_dof_z+base_dofs, : elev_dof_z+base_dofs], bin[: elev_dof_z+base_dofs]
            Ain = np.zeros((self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs+6, self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs+6))
            bin = np.zeros((self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs+6))
            Ain_elev, bin_elev = arm_utilities.joint_velocity_damper(ps_elev, pi_elev, n=elev_dof_z+elev_dof_rxy+base_dofs, q=armJointsElev[:5], qlim=qlim_h)

            Ain[:elev_dof_z+base_dofs+elev_dof_rxy, :elev_dof_z+base_dofs+elev_dof_rxy] = Ain_elev
            bin[:elev_dof_z+base_dofs+elev_dof_rxy] = bin_elev

            # Panda Arm Limits
            ps_arm = 0.1
            pi_arm = 0.9

            Ain_arm, bin_arm = arm_utilities.joint_velocity_damper(ps_arm, pi_arm, n=self.arm_dof, q=armJointsElev[5:], qlim=qlim_h[:,5:])

            Ain[elev_dof_z+base_dofs+elev_dof_rxy: self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs, elev_dof_z+elev_dof_rxy+base_dofs: self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs] = Ain_arm
            bin[elev_dof_z+base_dofs+elev_dof_rxy: self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs] = bin_arm

            
            # Ain[: self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs, : self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs], bin[: self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs] = arm_utilities.joint_velocity_damper(ps=0.1, pi=0.9, n=self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs, q=armJointsElev, qlim=qlim_h)
            
            
            # Linear component of objective function: the manipulability Jacobian)
            # # Weight manipulability Jacobian so prismatic joint has less effect
            # k = 10
            pandaBaseJacobianOG = self.panda_base_arm.jacobe(q=armJointsElev, start=self.panda_base_arm.links[5])
            pandaBaseArmM = -self.panda_base_arm.jacobm(q=armJointsElev, start=self.panda_base_arm.links[7])
            pandaBaseElevM = -self.panda_base_arm.jacobm(q=armJointsElev, start=self.panda_base_arm.links[5])
            # pandaBaseElevM = -self.panda_arm.jacobm(q=arm_joints)
            # print(f"pandaBaseElevM: {pandaBaseElevM}")
            # return
            # # # print(f"modJacobian: {pandaBaseJacobian}")
            # # pandaBaseJacobianOG = copy.deepcopy(pandaBaseJacobian)

            # pandaBaseArmJacobM[0,0] = pandaBaseArmJacobM[0,0]*k # only multiplying the first element by scalar
            # # pandaBaseArmJacobM = -self.panda_base_arm.jacobm(J=pandaBaseJacobian, start=self.panda_base_arm.links[4])


            c1 = np.concatenate(
                (np.zeros(base_dofs+elev_dof_z+elev_dof_rxy), pandaBaseArmM.reshape((self.arm_dof)), np.zeros(6))
            )  
            
            c2 = np.concatenate(
                (np.zeros(base_dofs+elev_dof_z), pandaBaseElevM.reshape((self.arm_dof+elev_dof_rxy)), np.zeros(6))
            ) 

            # e_norm = (spatial_error-np.min(np.asarray(self.spatialErrorList)))/ (np.max(np.asarray(self.spatialErrorList)) - np.min(np.asarray(self.spatialErrorList)))


            # print(f"e_norm: {e_norm}")
            c = 0.0*(c2-c1)+c1

            # Remove Manipulability Maximization
            # c = np.concatenate(
            #     (np.zeros(base_dofs+elev_dof_z+self.arm_dof+elev_dof_rxy), np.zeros(6))
            # )  
             
            # α = np.exp(-1*m/(dedt))
            # Get last g data points
            g = 100
            spatialErrorRecent = np.asarray(self.spatialErrorList[-g:])
            sETimeRecent = np.asarray(self.sETimeList[-g:])

            # if len(spatialErrorRecent) < g:
            #     α = 1
                
            # else:
            #     # spatialErrorRecent -= spatialErrorRecent[0]
            #     # sETimeRecent -= sETimeRecent[0]
            #     # print(f"spatialErrorRecent {spatialErrorRecent}")
            #     # print(f"sETimeRecent: {sETimeRecent}")
            #     # dedt = np.diff(np.asarray(spatialErrorRecent))/np.diff(np.asarray(sETimeRecent))
            #     # dedt_median = abs(np.average(dedt))
            #     dedt = abs(np.polyfit(sETimeRecent, spatialErrorRecent, 1)[0])
            #     # print(f"dedt {dedt}")
            #     m = .001 # input scalar
            #     α = np.exp(-1*m/(dedt))
            #     if α == 0:
            #         # α = 1
            #         print(f"Weird case alpha 0: dedt: {dedt}")
            #         break
            #     print(f"Alpha {α}: dedt: {dedt}")
            #     c = α * c #  Adjust alpha based on if we get stuck in a local minima 

            

            
            
            # print(f"dedt_median: {dedt_median}")
            # print(f"Alpha: {α}")
                         
            # Get base to face end-effector
            kε = 0.5
            b2e_T = self.panda_arm.fkine(q=arm_joints[2:]).A
            θε = np.arctan2(b2e_T[1, -1], b2e_T[0, -1])
            ε = kε * θε
            c[0] = -ε
           

            # The lower and upper bounds on the joint velocity and slack variable
            lb = -np.r_[self.panda_base_arm.qdlim[: self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs], 10 * np.ones(6)]
            ub =  np.r_[self.panda_base_arm.qdlim[: self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs], 10 * np.ones(6)]

            # Define Q
            Q = np.eye(self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs + 6)
            k_a = 0.4
            Q[: self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs, : self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs] *= k_a # apply k_a to arm joints in Q
            Q[3:5,3:5] *= 10 # elevator has less importance, higher number = harder to move
            # Q[:base_dofs, :base_dofs] *= 1.0/spatial_error
            Q[:base_dofs+elev_dof_z, :base_dofs+elev_dof_z] *= 1.0/spatial_error
            # Slack Q
            Q[self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs :, self.arm_dof+elev_dof_z+elev_dof_rxy+base_dofs :] = (1.0 / spatial_error) * np.eye(6)


            # Solve QP for q_dot and slack variable
            # print(f"Ain {Ain.shape}")
            # print(f"bin {bin.shape}")
            # print(f"Q: {Q.shape}")
            # print(f"Aeq: {Aeq.shape}")
            # print(f"beq: {beq.shape}")
            # print(f"lb {lb.shape}")
            # print(f"ub: {ub.shape}")

            qd = solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver='cvxopt')

            ## Send motion to base & elevator
            # Transform Qd into sent joint commands
            # Turn Virtual Joints into motion
            qd_bθ = qd[0]
            qd_bδ = qd[1]
            
            qd_bε = qd[2]
            qd_bεx = qd[3]
            qd_bεy = qd[4]
            
            w_l = (-qd_bθ*(self.lx + self.ly) + qd_bδ) / self.R
            w_r = (2*qd_bδ/self.R) - w_l

            (Vx_base, Vy_base, W_z_base) = arm_utilities.mecanum_FK(w_l, w_r, w_l, w_r, self.R, self.lx, self.ly)

            euler = euler_from_quaternion(w2b_orient) # translate to euler
            curr_val = euler[2] # z rotation

            if curr_val < 0:
                curr_val = curr_val + 2*np.pi # maps from 0-2pi now


            # Create 2D rotation matrix around local chassis Z axis (rotation)
            R_z = np.array(([np.cos(curr_val), -np.sin(curr_val), 0],
                            [np.sin(curr_val), np.cos(curr_val), 0],
                            [0, 0, 1]))

            # Rotate Robot XY velocities to World XY Velocities 
            worldV_XY = R_z@np.array(([Vx_base, Vy_base, 0]))

            # Base Publishers
            self.Vx_world.publish(worldV_XY[0])
            self.Vy_world.publish(worldV_XY[1])
            self.W_z_world.publish(W_z_base)

            # Elevator Publishers
            self.Vz_world.publish(qd_bε)
            self.rx_elev.publish(qd_bεx)
            self.ry_elev.publish(qd_bεy)

            # Arm Publisher
            arm_qd = [qd[5], qd[6], qd[7], qd[8], qd[9], qd[10], qd[11]]
            self.sendArmVels(arm_qd)

            # print(f"Error: {spatial_error}")

            # Publish manipulabulity metric (Yoshikawa)
            # Use this to record data and evaluate performance
            m = self.panda_arm.manipulability(q=arm_joints[2:])
            mb = self.yoshikawa(J=pandaBaseJacobianOG)
            # mModJacobian = self.yoshikawa(J=pandaBaseJacobian)
            # print(f"modmanip: {mModJacobian}")

            self.manipPub.publish(m)
            self.manipBasePub.publish(mb)
            # self.modJacPub.publish(mModJacobian)

            
        # Stop moving! 
        self.sendArmVels([0,0,0,0,0,0,0])
        self.Vx_world.publish(0)
        self.Vy_world.publish(0)
        self.Vz_world.publish(0)
        self.W_z_world.publish(0)
        self.rx_elev.publish(0)
        self.ry_elev.publish(0)

        # Update reached goal param
        self.reached_goal.data = True
        rospy.set_param('/goal_reached', self.reached_goal.data)
        
        

if __name__ == "__main__":
    
    ## Set homogenmous Transformation Matrix as goal coordinate system wrt to world coordinate system
    # goalT = np.array(([1, 0, 0, 2.3],
    #                   [0, -1, 0, 0.0],
    #                   [0, 0, -1, 1.95],
    #                   [0, 0, 0, 1]))

    # Hard goal 
    goalT = np.array(([0, 0, 1, 2.3],
                      [-1, 0, 0, 0.3],
                      [0, -1, 0, 0.5],
                      [0, 0, 0, 1]))
    holistic = holistic_control(goalT=goalT)
    holistic.main_loop(goalT)
    # rospy.spin()
