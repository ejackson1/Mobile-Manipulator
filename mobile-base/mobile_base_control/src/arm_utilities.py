#!/usr/bin/env python3

import numpy as np

class arm_utilities:
    def __init__(self) -> None:
        pass

    @staticmethod
    def skew(vector):
        Skew = np.array(([0, -float(vector[2]), float(vector[1])],
                            [float(vector[2]), 0, -float(vector[0])],
                            [-float(vector[1]), float(vector[0]), 0]))
        return Skew    
    
    @staticmethod
    def t2ADt(T):
        """
        T is a SE(3) Transformation Matrix
        """
        R = T[0:3,0:3]
        P = T[0:3, -1]
        top_ = np.hstack((R, np.zeros((3,3))))
        bot_ = np.hstack((arm_utilities.skew(P)@R, R))

        ADt = np.vstack((top_,bot_))

        # print(f"ADt: {ADt}")
        return ADt

    @staticmethod
    def screwR(w, theta):
        """
        Returns exp [w]theta -> the rotation matrix
        """
        skewW = arm_utilities.skew(w)
        return np.eye(3) + np.sin(theta)*skewW + (1 - np.cos(theta))*skewW@skewW


    @staticmethod
    def screw2T(Si, theta):
        skewW = arm_utilities.skew(Si[0:3])
        R = arm_utilities.screwR(Si[0:3], theta)
        p = (np.eye(3)*theta+(1-np.cos(theta)*skewW)+(theta-np.sin(theta))*skewW@skewW)@Si[3:6]

        # print(f"p: {p.reshape((3,1))}")
        # print(f"R: {R}")
        top_ = np.hstack((R, p.reshape((3,1))))
        bot_ = np.array(([0, 0, 0, 1]))

        return np.vstack((top_, bot_))


    @staticmethod
    def mJacobian(S, arm_dof, arm_joint_states):
        mJacob = np.zeros((6, arm_dof))
        for i in range(arm_dof):
            S_i = S[:, i]
            theta = arm_joint_states[i]
            T_i = arm_utilities.screw2T(S_i, theta)
            J_i = arm_utilities.t2ADt(T_i)@S_i
            mJacob[:,i] = J_i

        # print(f"mJacob {mJacob}")
        # Jw; Js in this form. Need to flip
        return mJacob


    @staticmethod
    def fkine(S, M, arm_dof, arm_joint_states):
        prev_T = np.eye(4)
        for i in range (arm_dof):
            S_i = S[:, i]
            T = arm_utilities.screw2T(S_i, arm_joint_states[i])@prev_T
            prev_T = T

        T = T@M

        return T


    @staticmethod
    def hessian0(J0, arm_dof):
        r"""
        Manipulator Hessian
        The manipulator Hessian tensor maps joint acceleration to end-effector
        spatial acceleration, expressed in the world-coordinate frame. This
        function calulcates this based on the ETS of the robot.
        
        One of J0 or q
        is required. Supply J0 if already calculated to save computation time
        :param q: The joint angles/configuration of the robot.
        :type q: ArrayLike
        :param J0: The manipulator Jacobian in the 0 frame
        :param tool: a static tool transformation matrix to apply to the
            end frame, defaults to None
        :return: The manipulator Hessian in 0 frame
        This method computes the manipulator Hessian in the base frame.  If
        we take the time derivative of the differential kinematic relationship
        .. math::
            \nu    &= \mat{J}(\vec{q}) \dvec{q} \\
            \alpha &= \dmat{J} \dvec{q} + \mat{J} \ddvec{q}
        where
        .. math::
            \dmat{J} = \mat{H} \dvec{q}
        and :math:`\mat{H} \in \mathbb{R}^{6\times n \times n}` is the
        Hessian tensor.
        The elements of the Hessian are
        .. math::
            \mat{H}_{i,j,k} =  \frac{d^2 u_i}{d q_j d q_k}
        where :math:`u = \{t_x, t_y, t_z, r_x, r_y, r_z\}` are the elements
        of the spatial velocity vector.
        Similarly, we can write
        .. math::
            \mat{J}_{i,j} = \frac{d u_i}{d q_j}
        :references:
            - Kinematic Derivatives using the Elementary Transform
              Sequence, J. Haviland and P. Corke
        """

        def cross(a, b):
            x = a[1] * b[2] - a[2] * b[1]
            y = a[2] * b[0] - a[0] * b[2]
            z = a[0] * b[1] - a[1] * b[0]
            return np.array([x, y, z])

        n = arm_dof

        H = np.zeros((n, 6, n))

        for j in range(n):
            for i in range(j, n):

                H[j, :3, i] = cross(J0[3:, j], J0[:3, i])
                H[j, 3:, i] = cross(J0[3:, j], J0[3:, i])

                if i != j:
                    H[i, :3, j] = H[j, :3, i]

        return H
        
    @staticmethod
    def yoshikawa_manip(J):
        if J.shape[0] == J.shape[1]:
            return abs(np.linalg.det(J))
        else:
            m2 = np.linalg.det(J @ J.T)
            return np.sqrt(abs(m2))

    @staticmethod
    def joint_velocity_damper(ps=0.05, pi=0.1, n=None, q=None, qlim=None, gain=1.0):
        """
        Formulates an inequality contraint which, when optimised for will
        make it impossible for the robot to run into joint limits. Requires
        the joint limits of the robot to be specified. See examples/mmc.py
        for use case

        :param ps: The minimum angle (in radians) in which the joint is
            allowed to approach to its limit
        :type ps: float
        :param pi: The influence angle (in radians) in which the velocity
            damper becomes active
        :type pi: float
        :param n: The number of joints to consider. Defaults to all joints
        :type n: int
        :param gain: The gain for the velocity damper
        :type gain: float

        :returns: Ain, Bin as the inequality contraints for an optisator
        :rtype: ndarray(6), ndarray(6)
        """

        Ain = np.zeros((n, n))
        Bin = np.zeros(n)

        for i in range(n):
            # print(f"q{i} - qlim0,{i} = {q[i] - qlim[0, i]}")
            # print(f"qlim1,{i} - q{i} = {qlim[1, i] - q[i]}")
            if q[i] - qlim[0, i] <= pi:
                Bin[i] = -gain * (((qlim[0, i] - q[i]) + ps) / (pi - ps))
                Ain[i, i] = -1
            if qlim[1, i] - q[i] <= pi:
                Bin[i] = gain * ((qlim[1, i] - q[i]) - ps) / (pi - ps)
                Ain[i, i] = 1
        # print(f"Ainfunc: {Ain}")
        return Ain, Bin

    @staticmethod
    def mecanum_FK(w_fl, w_fr, w_rl, w_rr, r, lx, ly):
        """
        w_fl, w_fr, w_rl, w_rr -> wheel velocities
        Returns Vx, Vy, and W_z
        """
        a_ = np.array(([1, 1, 1, 1],
                      [-1, 1, 1, -1],
                      [-1/(lx+ly), 1/(lx+ly), -1/(lx+ly), 1/(lx+ly)]))
        b_ = np.array(([w_fl], [w_fr], [w_rl], [w_rr]))
        FK = r/4*a_@b_
        Vx = float(FK[0])
        Vy = float(FK[1])
        W_z = float(FK[2])

        return (Vx, Vy, W_z)
    @staticmethod
    def mecanum_IK(v_x, v_y, w_z, r, lx, ly):
        """
        """
        a_ = np.array([1, -1, -(lx+ly)],
                      [1, 1, (lx+ly)],
                      [1, 1, -(lx+ly)],
                      [1, -1, (lx+ly)])
        b_ = np.array([v_x], [v_y], [w_z])

        IK = 1/r@a_@b_

        return IK