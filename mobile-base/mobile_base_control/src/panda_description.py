#!/usr/bin/env python3

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

        # Base links
        l_b = Link(ET.tz(0.284248) ,name="base_link", parent=None) 
        l_3 = Link(ET.Rz(), name="base_linkθ", parent=l_b)
        l_2 = Link(ET.tx(), name="base_linkδ", parent=l_3, qlim=[-99999,99999])
        
        # Transform to elevator
        l_1 = Link(ET.tx(0.117535817-53.87925*mm) * ET.tz(1-0.187907), name="base2elev", parent=l_2) # subtracted value in ET.tz is expirementally found from sim value - fkine.A
        
        # Elevator Z prismatic
        l_e = Link(ET.tz(), name="base_linkεz", parent=l_1, qlim=[-0.25,0.25])

        # Rotation in X and Y axis revolute
        l_erx = Link(ET.Rx(), name="base_linkεx", parent=l_e, qlim=[-0.3, 0.3])
        l_ery = Link(ET.Ry(), name="base_linkεy", parent=l_erx, qlim=[-0.3, 0.3])

        # Static X translation to get to base of arm from elevator 
        l_x = Link(ET.tx(53.87925*mm), name="elev2arm", parent=l_ery)

        # Panda Arm
        l0 = Link(ET.tz(0.333) * ET.Rz(), name="link0", parent=l_x)        
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

        elinks = [l_b, l_3, l_2, l_1, l_e, l_erx, l_ery, l_x, l0, l1, l2, l3, l4, l5, l6, ee]
        self.qdlim = np.array(
                [0.5, 0.5, 0.1, 0.1, 0.1, 2.1750, 2.1750,  2.1750, 2.1750, 2.6100, 2.6100, 2.6100, 3.0, 3.0]
        )
        super(Panda_Modified, self).__init__(elinks, name="Panda Modified", manufacturer="WPI & Franka Emika")

        # self.qr = np.array([0, -0.3, 0, -2.2, 0, 2.0, np.pi / 4])
        # self.qz = np.zeros(7)

        # self.addconfiguration("qr", self.qr)
        # self.addconfiguration("qz", self.qz)


if __name__ == "__main__":  # pragma nocover

    robot = Panda_Modified()
    print(robot)
