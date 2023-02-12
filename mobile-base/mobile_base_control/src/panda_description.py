# #!/usr/bin/env python3

# import rospy
# import numpy as np
# import sys
# import os

# # from setuptools import setup, find_packages
# # setup(name= 'mobile_base_control', packages=find_packages())


# from arm_utilities import arm_utilities

# # Try to find a way to put panda_description in the panda_arm directory 
# class panda_description:
#     L0_12 = 0.333
#     L12_3 = 0.316
#     L3_4 = 0.088
#     L4_5 = 0.384
#     L6_7 = 0.088
#     L7_F = 0.107

#     W1 = np.array(([0], [0], [1]))
#     P1 = np.array(([0], [0], [L0_12]))
#     V1 = arm_utilities.skew(-W1)@P1
#     S1 = np.r_[W1, V1]

#     W2 = np.array(([0], [1], [0]))
#     P2 = np.array(([0], [0], [L0_12]))
#     V2 = arm_utilities.skew(-W2)@P2
#     S2 = np.r_[W2, V2]

#     W3 = np.array(([0], [0], [1]))
#     P3 = np.array(([0], [0], [L0_12+L12_3]))
#     V3 = arm_utilities.skew(-W3)@P3
#     S3 = np.r_[W3, V3]

#     W4 = np.array(([0], [-1], [0]))
#     P4 = np.array(([L3_4], [0], [L0_12+L12_3]))
#     V4 = arm_utilities.skew(-W4)@P4
#     S4 = np.r_[W4, V4]

#     W5 = np.array(([0], [0], [1]))
#     P5 = np.array(([0], [0], [L0_12+L12_3+L4_5]))
#     V5 = arm_utilities.skew(-W5)@P5
#     S5 = np.r_[W5, V5]

#     W6 = np.array(([0], [-1], [0]))
#     P6 = np.array(([0], [0], [L0_12+L12_3+L4_5]))
#     V6 = arm_utilities.skew(-W6)@P6
#     S6 = np.r_[W6, V6]

#     W7 = np.array(([0], [0], [-1]))
#     P7 = np.array(([L6_7], [0], [L0_12+L12_3+L4_5]))
#     V7 = arm_utilities.skew(-W7)@P7
#     S7 = np.r_[W7, V7]

#     WF = np.array(([0], [0], [-1]))
#     PF = np.array(([L6_7], [0], [L0_12+L12_3+L4_5-L7_F]))
#     VF = arm_utilities.skew(-WF)@PF
#     SF = np.r_[WF, VF]

#     S = np.c_[S1, S2, S3, S4, S5, S6, S7]


#     M = np.array(([1, 0, 0, L6_7],
#                   [0, -1, 0, 0],
#                   [0, 0, -1, L0_12+L12_3+L4_5-L7_F],
#                   [0, 0, 0, 1]))

#     def __init__(self):
#         pass
    

# # if __name__ == "__main__":
# #     panda = panda_description()


# #!/usr/bin/env python

# import numpy as np
# from spatialmath.base import trotz, transl
# from roboticstoolbox import DHRobot, RevoluteMDH, PrismaticMDH


# class Panda_Modified(DHRobot):
#     """
#     A class representing the Panda robot arm.

#     ``Panda()`` is a class which models a Franka-Emika Panda robot and
#     describes its kinematic characteristics using modified DH
#     conventions.

#     .. runblock:: pycon

#         >>> import roboticstoolbox as rtb
#         >>> robot = rtb.models.DH.Panda()
#         >>> print(robot)

#     .. note::
#         - SI units of metres are used.
#         - The model includes a tool offset.

#     :references:
#         - https://frankaemika.github.io/docs/control_parameters.html

#     .. codeauthor:: Samuel Drew
#     .. codeauthor:: Peter Corke
#     .. codeauthor:: Edward Jackson
#     """

#     def __init__(self):

#         # deg = np.pi/180
#         mm = 1e-3
#         tool_offset = (103) * mm

#         flange = (107) * mm
#         # d7 = (58.4)*mm

#         # This Panda model is defined using modified
#         # Denavit-Hartenberg parameters
#         L = [
#             PrismaticMDH(
#                 a=0.0,
#                 theta=0.0,
#                 alpha=np.pi/2,
#                 qlim=np.array([-99999, 99999]),
#                 m=90,
#                 I=[
#                     7.03370e-01,
#                     7.06610e-01,
#                     9.11700e-03,
#                     -1.39000e-04,
#                     1.91690e-02,
#                     6.77200e-03,
#                 ],
#                 G=1,

#             ),
#             RevoluteMDH(
#                 a=.30, # l2
#                 d=.30, # l1
#                 offset=np.pi/2,
#                 alpha=0.0,
#                 qlim=np.array([-99999, 99999]),
#                 m=90,
#                 I=[
#                     7.03370e-01,
#                     7.06610e-01,
#                     9.11700e-03,
#                     -1.39000e-04,
#                     1.91690e-02,
#                     6.77200e-03,
#                 ],
#                 G=1,
#             ),
#             ## Start of normal arm
#             RevoluteMDH(
#                 a=0.0,
#                 d=0.333,
#                 alpha=0.0,
#                 qlim=np.array([-2.8973, 2.8973]),
#                 m=4.970684,
#                 I=[
#                     7.03370e-01,
#                     7.06610e-01,
#                     9.11700e-03,
#                     -1.39000e-04,
#                     1.91690e-02,
#                     6.77200e-03,
#                 ],
#                 G=1,
#             ),
#             RevoluteMDH(
#                 a=0.0,
#                 d=0.0,
#                 alpha=-np.pi / 2,
#                 qlim=np.array([-1.7628, 1.7628]),
#                 m=0.646926,
#                 I=[
#                     7.96200e-03,
#                     2.81100e-02,
#                     2.59950e-02,
#                     -3.92500e-03,
#                     7.04000e-04,
#                     1.02540e-02,
#                 ],
#                 G=1,
#             ),
#             RevoluteMDH(
#                 a=0.0,
#                 d=0.316,
#                 alpha=np.pi / 2,
#                 qlim=np.array([-2.8973, 2.8973]),
#                 m=3.228604,
#                 I=[
#                     3.72420e-02,
#                     3.61550e-02,
#                     1.08300e-02,
#                     -4.76100e-03,
#                     -1.28050e-02,
#                     -1.13960e-02,
#                 ],
#                 G=1,
#             ),
#             RevoluteMDH(
#                 a=0.0825,
#                 d=0.0,
#                 alpha=np.pi / 2,
#                 qlim=np.array([-3.0718, -0.0698]),
#                 m=3.587895,
#                 I=[
#                     2.58530e-02,
#                     1.95520e-02,
#                     2.83230e-02,
#                     7.79600e-03,
#                     8.64100e-03,
#                     -1.33200e-03,
#                 ],
#                 G=1,
#             ),
#             RevoluteMDH(
#                 a=-0.0825,
#                 d=0.384,
#                 alpha=-np.pi / 2,
#                 qlim=np.array([-2.8973, 2.8973]),
#                 m=1.225946,
#                 I=[
#                     3.55490e-02,
#                     2.94740e-02,
#                     8.62700e-03,
#                     -2.11700e-03,
#                     2.29000e-04,
#                     -4.03700e-03,
#                 ],
#                 G=1,
#             ),
#             RevoluteMDH(
#                 a=0.0,
#                 d=0.0,
#                 alpha=np.pi / 2,
#                 qlim=np.array([-0.0175, 3.7525]),
#                 m=1.666555,
#                 I=[
#                     1.96400e-03,
#                     4.35400e-03,
#                     5.43300e-03,
#                     1.09000e-04,
#                     3.41000e-04,
#                     -1.15800e-03,
#                 ],
#                 G=1,
#             ),
#             RevoluteMDH(
#                 a=0.088,
#                 d=flange,
#                 alpha=np.pi / 2,
#                 qlim=np.array([-2.8973, 2.8973]),
#                 m=7.35522e-01,
#                 I=[
#                     1.25160e-02,
#                     1.00270e-02,
#                     4.81500e-03,
#                     -4.28000e-04,
#                     -7.41000e-04,
#                     -1.19600e-03,
#                 ],
#                 G=1,
#             ),
#         ]

#         tool = transl(0, 0, tool_offset) @ trotz(-np.pi / 4)

#         super().__init__(
#             L,
#             name="Panda",
#             manufacturer="Franka Emika",
#             meshdir="meshes/FRANKA-EMIKA/Panda",
#             tool=tool,
#         )

#         # self.qr = np.array([0, -0.3, 0, -2.2, 0, 2.0, np.pi / 4])
#         # self.qz = np.zeros(7)

#         # self.addconfiguration("qr", self.qr)
#         # self.addconfiguration("qz", self.qz)


# if __name__ == "__main__":  # pragma nocover

#     panda = Panda_Modified()
#     print(panda)

#!/usr/bin/env python

import numpy as np
from roboticstoolbox.robot.ET import ET
from roboticstoolbox.robot.ERobot import ERobot
from roboticstoolbox.robot.Link import Link


class Panda_Modified(ERobot):
    """
    Create model of Franka-Emika Panda manipulator

    panda = Panda() creates a robot object representing the Franka-Emika
    Panda robot arm. This robot is represented using the elementary
    transform sequence (ETS).

    ETS taken from [1] based on
    https://frankaemika.github.io/docs/control_parameters.html

    :references:
        - Kinematic Derivatives using the Elementary Transform
          Sequence, J. Haviland and P. Corke

    """

    def __init__(self):

        deg = np.pi / 180
        mm = 1e-3
        tool_offset = (103) * mm

        l_b = Link(ET.tz(0.284248) ,name="base_link", parent=None) # raise
        l_3 = Link(ET.Rz(), name="base_linkθ", parent=l_b)
        l_2 = Link(ET.tx(), name="base_linkδ", parent=l_3, qlim=[-99999,99999])
        l_1 = Link(ET.tx(0.117535817) * ET.tz(1-0.187907), name="base2arm", parent=l_2) # subtracted value in ET.tz is expirementally found from sim value - fkine.A


        l0 = Link(ET.tz(0.333) * ET.Rz(), name="link0", parent=l_1)

        l1 = Link(ET.Rx(-90 * deg) * ET.Rz(), name="link1", parent=l0)

        l2 = Link(ET.Rx(90 * deg) * ET.tz(0.316) * ET.Rz(), name="link2", parent=l1)

        l3 = Link(ET.tx(0.0825) * ET.Rx(90, "deg") * ET.Rz(), name="link3", parent=l2)

        l4 = Link(
            ET.tx(-0.0825) * ET.Rx(-90, "deg") * ET.tz(0.384) * ET.Rz(),
            name="link4",
            parent=l3,
        )

        l5 = Link(ET.Rx(90, "deg") * ET.Rz(), name="link5", parent=l4)

        l6 = Link(
            ET.tx(0.088) * ET.Rx(90, "deg") * ET.tz(0.107) * ET.Rz(),
            name="link6",
            parent=l5,
        )

        ee = Link(ET.tz(tool_offset) * ET.Rz(-np.pi / 4), name="ee", parent=l6)

        elinks = [l_b, l_3, l_2, l_1, l0, l1, l2, l3, l4, l5, l6, ee]
        self.qdlim = np.array(
            [0.5, 0.5, 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100, 3.0, 3.0]
        )
        super(Panda_Modified, self).__init__(elinks, name="Panda Modified", manufacturer="WPI & Franka Emika")

        # self.qr = np.array([0, -0.3, 0, -2.2, 0, 2.0, np.pi / 4])
        # self.qz = np.zeros(7)

        # self.addconfiguration("qr", self.qr)
        # self.addconfiguration("qz", self.qz)


if __name__ == "__main__":  # pragma nocover

    robot = Panda_Modified()
    print(robot)
