#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import numpy as np
import sys, select, termios, tty
from tf.transformations import euler_from_quaternion
from tf2_msgs.msg import TFMessage
from tf import TransformListener

## UTILIZE TO TURN PRISMATIC WORLD JOINTS INTO MECANUM MOTION ##
# Math based from https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html

class sim_mecanum_logic():
    def __init__(self):
        rospy.init_node("sim_mecanum_logic", anonymous=True)
        self.root_joint_name = "base_footprint"
        self.tf = TransformListener()

        base_controller_str = "/base/base_{0}_joint_controller/command"
        self.Vx_world = rospy.Publisher(base_controller_str.format('x'), Float64, queue_size=10)
        self.Vy_world = rospy.Publisher(base_controller_str.format('y'), Float64, queue_size=10)
        self.W_z_world = rospy.Publisher("/base/base_z_rotation_controller/command", Float64, queue_size=10)


    def mecanum_FK(self, w_fl, w_fr, w_rl, w_rr, r, lx, ly):
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

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


if __name__ == "__main__":
    sim_mecanum = sim_mecanum_logic()
    settings = termios.tcgetattr(sys.stdin)
    tf = TransformListener()
    # key_timeout = rospy.get_param("~key_timeout", 0.0)
    ## KEYS ##
    msg = """
    T Y U I
    G H J K

    CW / FORWARD:       T, G, I, K
    CCW / BACKWARD:     Y, H, U, J

    FRONT LEFT:     T, Y
    FRONT RIGHT:    U, I
    BACK LEFT:      G, H
    BACK RIGHT:     J, K 

    SPEED ON/OFF:   B, N  

    """
    
    # moveBindings Notion #
    # 'key': (speed, ccw/cw, on?)
    speed = 0.5 # 0.5 rad/s
    moveBindings = {
        't':(speed, 'cw'),
        'y':(speed, 'ccw'),
        'u':(speed, 'ccw'),
        'i':(speed, 'cw'),
        'g':(speed, 'cw'),
        'h':(speed, 'ccw'),
        'j':(speed, 'ccw'),
        'k':(speed, 'cw')        
        }
        

    
    # CAD CONSTANTS
    r, lx, ly = 0.5,1,1 # TODO fill in with CAD values

    # INITS
    w_fl, w_fr, w_rl, w_rr = 0,0,0,0

    print(msg)
    while True:
        # Keyboard input
        key = sim_mecanum.getKey()

        if key == '\x03': break
        # w_fl, w_fr, w_rl, w_rr, r, lx, ly
        elif key == 'b': speed = 0.5
        elif key == 'n': speed = 0

        elif key in moveBindings.keys():
            if moveBindings[key][1] == 'cw':
                dir_mod = -1
            else:
                dir_mod = 1
            
            # FRONT LEFT WHEEL
            if key == 't' or key == 'y':
                w_fl = speed * dir_mod
            # FRONT RIGHT WHEEL
            elif key == 'u' or key == 'i':
                w_fr = speed * dir_mod
            # BACK LEFT WHEEL
            elif key == 'g' or key== 'h':
                w_rl = speed * dir_mod
            # BACK RIGHT WHEEL
            elif key == 'j' or key == 'k':
                w_rr = speed * dir_mod
        
        (Vx, Vy, W_z) = sim_mecanum.mecanum_FK(w_fl, w_fr, w_rl, w_rr, r, lx, ly)    
        

        # print(f"Vx, Vy, Wz: {Vx, Vy, W_z}")
        print(f"w_fl {w_fl}, w_fr {w_fr}, w_rl {w_rl}, w_rr {w_rr}")
        print(f"Speed {speed}")

        try:
            root_joint_name = "base_footprint"
            position, quaternion = tf.lookupTransform("world", root_joint_name, rospy.Time())

            if position != [0.0,0.0,0.0]:    
                # print(position)

                euler = euler_from_quaternion(quaternion) # translate to euler
                curr_val = euler[2] # z rotation
                # print(f"Euler Angles: {curr_val}")

                if curr_val < 0:
                    curr_val = curr_val + 2*np.pi # maps from 0-2pi now

                # Convert angle to map to X Y world coordinates
                y_w = np.sin(curr_val)
                x_w = np.cos(curr_val)


                # Create 2D rotation matrix around local chassis Z axis (rotation)
                R_z = np.array(([np.cos(curr_val), -np.sin(curr_val), 0],
                                [np.sin(curr_val), np.cos(curr_val), 0],
                                [0, 0, 1]))

                # Rotate Robot XY velocities to World XY Velocities 
                worldV_XY = R_z@np.array(([Vx, Vy, 0]))
                

                # Publishing world velocities and rotations
                sim_mecanum.Vx_world.publish(worldV_XY[0])
                sim_mecanum.Vy_world.publish(worldV_XY[1])
                sim_mecanum.W_z_world.publish(W_z)


        except Exception as e:
            print("Failed lookup, retrying")
            pass




        

