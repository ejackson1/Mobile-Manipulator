#!/usr/bin/env python3
from threading import current_thread
import rospy
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from tf import TransformListener
from tf.transformations import euler_from_quaternion
import numpy as np


class Follower(object):
    def __init__(self):
        rospy.init_node("base_follower")
        self.tf = TransformListener()

        self.arm_controllers = []
        self.prev_value = [0,0,0]
        self.z_rot_diff = 0
        self.z_rot_prev = 0
        self.z_rot_global = 0 
        self.rot_counter = 0

        arm_name = rospy.get_param("arm/name")
        print("arm: " + arm_name)
        arm_namespace = "panda" if arm_name=="panda" else "my_gen3"
        self.root_joint_name = "base_footprint" # "panda_link0" if arm_name=="panda" else "base_footprint" # idk why the panda needs the kortex base link but whatever

        arm_controller_str = "/" + arm_namespace + "/" + arm_name + "_{0}_joint_controller/command"

        self.arm_controllers.append(rospy.Publisher(arm_controller_str.format('x'), Float64, queue_size=10))
        self.arm_controllers.append(rospy.Publisher(arm_controller_str.format('y'), Float64, queue_size=10))
        self.arm_controllers.append(rospy.Publisher(arm_controller_str.format('z'), Float64, queue_size=10))
        self.arm_controllers.append(rospy.Publisher("/" + arm_namespace + "/" + arm_name + "_z_rotation_controller/command", Float64, queue_size=10))

        rospy.Subscriber("tf/", TFMessage, self.callback)

        rospy.spin()

    
    def quadrantCheck(self, angle):
        """
        This is a helper function designed to quickly evaluate which quadrant an angle in -pi,pi is in
        """
        if angle > 0:
            if angle > np.pi/2:
                return 2
            else:
                return 1
        else:
            if angle < -np.pi/2:
                return 3
            else:
                return 4
    
    def callback(self, data):
        try:
            # get position and quaternion between world and base
            position, quaternion = self.tf.lookupTransform("world", self.root_joint_name, rospy.Time())
            
            if position != [0.0,0.0,0.0]:    
                # print(position)

                euler = euler_from_quaternion(quaternion) # translate to euler
                euler = np.asarray(euler)
                
                quadrantCurrent = self.quadrantCheck(euler[2])
                quadrantPrev    = self.quadrantCheck(self.z_rot_prev)
                edgeCase = False

                # Check if we cross the -pi,pi x axis line or the 0 line! We handle this as an edge case
                if quadrantCurrent!= quadrantPrev:
                    if quadrantCurrent == 3 and quadrantPrev == 2:
                        # We've crossed the -pi,pi border going CCW
                        self.z_rot_diff = abs(euler[2]) + self.z_rot_prev - 2*np.pi
                        edgeCase = True

                    elif quadrantCurrent == 2 and quadrantPrev == 3:
                        # We've crossed the -pi,pi border going CW
                        self.z_rot_diff = -(euler[2] + abs(self.z_rot_prev) - 2*np.pi)
                        edgeCase = True
                    
                    elif quadrantCurrent == 1 and quadrantPrev == 4:
                        # We've crossed the 0 border going CCW
                        self.z_rot_diff = euler[2] + abs(self.z_rot_prev) 
                        edgeCase = True
                        

                    elif quadrantCurrent == 4 and quadrantPrev == 1:
                        self.z_rot_diff = -(abs(euler[2]) + self.z_rot_prev)
                        edgeCase = True
                        
                

                ## Normal operation
               
                if quadrantCurrent in [1,2] and quadrantPrev in [1,2] and edgeCase is False: # both in Quadrants I or II
                    # Determine direction
                    if euler[2] > self.z_rot_prev: # moving CCW
                        self.z_rot_diff = euler[2] - self.z_rot_prev
                    else: ## moving CW
                        self.z_rot_diff = -(self.z_rot_prev - euler[2])
                elif quadrantCurrent in [3,4] and quadrantPrev in [3,4] and edgeCase is False:
                    # Determine direction
                    if euler[2] > self.z_rot_prev: # moving CCW
                        self.z_rot_diff = abs(self.z_rot_prev) - abs(euler[2])
                    else: # moving CW
                        self.z_rot_diff = -(abs(euler[2])- abs(self.z_rot_prev))
                
                else:
                    print("Weird edge case ran")
                    print(f"Quadrants: {quadrantCurrent, quadrantPrev}")
                    print(f"Angles: {euler[2], self.z_rot_prev}")

               
            
                self.rot_counter += self.z_rot_diff
                # print(f"rot_counter {self.rot_counter}")


                ## TODO Work on fixing the drift
                ## Fix drift 

                # Send positions!
                

                curr_val = (position[0], position[1], euler[2]) # x, y pos and z rotation
                # print(curr_val)

                # print(f"euler: {euler}")
                self.arm_controllers[0].publish(curr_val[0]) # x 
                self.arm_controllers[1].publish(curr_val[1]) # y
                self.arm_controllers[3].publish(self.rot_counter) # z_rot
                self.z_rot_prev = euler[2]
                # self.arm_controllers[2].publish(1)
                
                # self.prev_value = curr_val
        except Exception as e:
            print(e)
            pass
        

if __name__ == '__main__':
    try:
        d = Follower()
    except rospy.ROSInterruptException:
        print("Error following base")
        
