#!/usr/bin/env python3

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import rospy
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from tf import TransformListener
from tf.transformations import euler_from_quaternion
import numpy as np
import threading


## Take in User inputs

msg = """
Control Your Turtlebot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
CTRL-C to
"""

# wrt to chassis_link
# i: forward
# k: backwards
# j: left
# l: right
# u: rotate ccw
# o: rotate cw

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

class sim_mecanum_teleop():
    def __init__(self, rate):
        self.tf = TransformListener()
        self.root_joint_name = "base_footprint"
        rospy.Subscriber("tf/", TFMessage, self.callback)
        rospy.spin()
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None
        
        self.base_controllers = []
        self.arm_controllers = []

        base_controller_str = "/base/base_{0}_joint_controller/command"

        self.base_controllers.append(rospy.Publisher(base_controller_str.format('x'), Float64, queue_size=10))
        self.base_controllers.append(rospy.Publisher(base_controller_str.format('y'), Float64, queue_size=10))
        self.base_controllers.append(rospy.Publisher(base_controller_str.format('z'), Float64, queue_size=10))
        self.base_controllers.append(rospy.Publisher("/base/base_z_rotation_controller/command", Float64, queue_size=10))

        self.start()

    def callback(self, data):
        try:
            # get position and quaternion between world and base
            position, quaternion = self.tf.lookupTransform("world", self.root_joint_name, rospy.Time())

            if position != [0.0,0.0,0.0]:    
                # print(position)

                euler = euler_from_quaternion(quaternion) # translate to euler
                curr_val = euler[2] # z rotation
                # print(f"Euler Angles: {curr_val}")

                if curr_val < 0:
                    curr_val = curr_val + 2*np.pi # maps from 0-2pi now


                # Convert angle to map to X Y world coordinates
                self.y_w = np.sin(curr_val)
                self.x_w = np.cos(curr_val)


        except Exception as e:
            pass
    
    
    
    def publish_to_all(self, twist):
        print(f"Twist {twist}")
        for i in range(4):
            modifier = twist[4] if i<3 else twist[5] # velocity or turn speed

            self.base_controllers[i].publish(twist[i]*modifier)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .2
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    sim = sim_mecanum_teleop()
    key_timeout = rospy.get_param("~key_timeout", 0.0)

    try:
        while(1):
            key = sim.getGet(key_timeout)
            if key == 'i':
                print("Moving Forward")
            elif key == 'k':
                print("Moving Backwards")
            elif key =='j':
                print("Strafing left")
            elif key == 'l':
                print("Strafing right")






    except rospy.ROSInterruptException:
        print("Error following base")








