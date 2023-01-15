#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import time
import sys, tty, select, termios
import threading

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class TeleOpDemo(threading.Thread):
    def __init__(self, rate): 
        super(TeleOpDemo, self).__init__()
        
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

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        
        # set values (to be published in other thread)
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn

        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into custom twist tuple
            twist = (self.x, self.y, self.z, self.th, self.speed, self.turn)

            self.condition.release()

            # Publish
            self.publish_to_all(twist)

        # Publish stop message when thread exits.
        self.publish_to_all((0,0,0,0,0,0))
    
    def publish_to_all(self, twist):
        print(f"Twist {twist}")
        for i in range(4):
            modifier = twist[4] if i<3 else twist[5] # velocity or turn speed

            self.base_controllers[i].publish(twist[i]*modifier)



def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    
    rospy.init_node("teleop")
    settings = termios.tcgetattr(sys.stdin)

    speed = rospy.get_param("~speed", 2)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = TeleOpDemo(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        time.sleep(2)
        pub_thread.update(x, y, z, th, speed, turn)

        while(1):
            key = getKey(key_timeout)

            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break
 
            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(f"Exception: {e}")

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)