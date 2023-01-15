#!/usr/bin/env python3

import rospy
import smach
import time
import threading
import queue
from task_manager.msg import task
from service_interface import ServiceInterface


# define Idle State
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['swap','task_progress', 'shutdown'],
                             output_keys=['task_progress_out'])

        self.mutex = threading.Lock()
        self.received_task = None
        self.task_queue = queue.Queue()

        task_queue_subscriber = rospy.Subscriber('/task_topic', task, self.add_to_queue)
        

    def add_to_queue(self, msg):
        self.mutex.acquire()
        self.task_queue.put(msg)
        rospy.loginfo('Added new ' + task_type_dict[msg.task_type] + ' task to queue')

        self.mutex.release()
        pass

    def execute(self, userdata):
        global current_task
        rospy.loginfo('Sitting in Idle state')
        waiting_for_task = True

        # spin while waiting for next task
        while waiting_for_task:
            self.mutex.acquire()
            if not self.task_queue.empty(): # if queue is not empty
                waiting_for_task = False
                
                current_task = self.task_queue.get() # pop from queue
                rospy.loginfo("--- Current task is " + task_type_dict[current_task.task_type] + " task ---")
            self.mutex.release()
        
        # after receiving task, all progress is reset
        userdata.task_progress_out = {
            'base_moved': False,
            'elevator_moved': False,
            'arm_moved': False,
        }

        if current_task.task_type != current_task.TASK_SHUTDOWN:
            if current_task.task_type == current_task.TASK_SWAP:
                return 'swap'
            else:
                return 'task_progress'
        else:
            return 'shutdown'

class Task_Progress(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['idle', 'move_base', 'move_arm', 'move_elevator'],
                             input_keys=['task_progress_in'],
                             output_keys=['task_progress_out', 'home_position_out'])

    def execute(self, userdata):
        # rospy.loginfo('Passing through Task_Progress state')

        # print("    " + str(userdata.task_progress_in))

        if all(userdata.task_progress_in.values()): # if all task components are complete
            return 'idle'

        userdata.task_progress_out = userdata.task_progress_in
        userdata.home_position_out = False

        # check arm in home pos (TODO: service)
        if not service_interface.is_arm_in_home_pos():
            print("----------ARM NOT IN HOME POS, MOVING TO HOME------------")
            userdata.home_position_out = True
            return 'move_arm'

        # check elevator in home pos (TODO: service)
        if False:
            userdata.home_position_out = True
            return 'move_elevator'

        # check if base has moved yet
        if userdata.task_progress_in['base_moved']:
            return 'check_elevator'
        else:
            return 'move_base'

class Base(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['check_elevator'],
                             input_keys=['task_progress_in'],
                             output_keys=['task_progress_out'])
    
    def execute(self, userdata):
        global current_task
        # rospy.loginfo('Moving base in Base state')

        # TODO: move base service
        # use current_task.base_orientation 
        time.sleep(3)

        # set base as moved in task progress
        d = userdata.task_progress_in
        d['base_moved'] = True
        userdata.task_progress_out = d

        return 'check_elevator'

class Elevator_Goal_Check(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['move_elevator', 'check_arm'],
                             input_keys=['task_progress_in'], 
                             output_keys=['task_progress_out', 'home_position_out'])

    def execute(self, userdata):
        # rospy.loginfo('Checking if elevator needs to be moved')

        userdata.task_progress_out = userdata.task_progress_in
        userdata.home_position_out = False
        
        if userdata.task_progress_in['elevator_moved']:
            return 'check_arm'
        else:
            return 'move_elevator'

class Elevator(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['check_arm', 'task_progress'],
                             input_keys=['task_progress_in', 'home_position_in'], 
                             output_keys=['task_progress_out'])
    
    def execute(self, userdata):
        global current_task
        # rospy.loginfo('Moving jacks in Elevator state')

        # if elevator needs to be homed, home it and go back to task progress
        if userdata.home_position_in:
            #TODO: elevator home service
            time.sleep(3)
            return 'task_progress'

        # TODO: move elevator service
        # use current_task.elevator_height 
        time.sleep(3)

        # set elevator as moved in task progress
        d = userdata.task_progress_in
        d['elevator_moved'] = True
        userdata.task_progress_out = d

        return 'check_arm'

class Arm_Goal_Check(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['move_arm', 'task_progress'],
                             input_keys=['task_progress_in'], 
                             output_keys=['task_progress_out', 'home_position_out'])

    def execute(self, userdata):
        # rospy.loginfo('Checking if arm needs to be moved')

        userdata.task_progress_out = userdata.task_progress_in
        userdata.home_position_out = False
        
        if userdata.task_progress_in['arm_moved']:
            return 'task_progress'
        else:
            return 'move_arm'

class Arm(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['task_progress'],
                             input_keys=['task_progress_in', 'home_position_in'], 
                             output_keys=['task_progress_out'])
    
    def execute(self, userdata):
        global current_task, service_interface
        # rospy.loginfo('Moving arm in Arm state')

        # if arm needs to be homed, home it and go back to task progress
        if userdata.home_position_in:
            service_interface.home_arm()
            return 'task_progress'

        # TODO: move arm service
        # TODO: move gripper service
        service_interface.move_arm(current_task.task_type, current_task.arm_orientation, current_task.gripper_closed)

        # set arm as moved in task progress
        d = userdata.task_progress_in
        d['arm_moved'] = True
        userdata.task_progress_out = d

        return 'task_progress'


if __name__ == '__main__':
    rospy.init_node('robot_state_machine')

    # global variables (to be used in various state classes)
    service_interface = ServiceInterface()
    current_task: task = None
    task_type_dict = {0: "Pick", 1: "Home", 2: "Swap", 3: "Shutdown"}

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['SHUTDOWN']) # add ERROR state?
    sm.userdata.task_progress = {
        'base_moved': False,
        'elevator_moved': False,
        'arm_moved': False,
    }
    sm.userdata.home_position = False

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'swap':'SHUTDOWN', 
                                            'task_progress':'TASK_PROGRESS',
                                            'shutdown':'SHUTDOWN'},
                               remapping={'task_progress_out':'task_progress'})
        smach.StateMachine.add('TASK_PROGRESS', Task_Progress(), 
                               transitions={'idle':'IDLE', 
                                            'move_base':'BASE',
                                            'move_arm':'ARM',
                                            'move_elevator': 'ELEVATOR'},
                               remapping={'task_progress_in':'task_progress',
                                          'task_progress_out':'task_progress',
                                          'home_position_out':'home_position'})


        smach.StateMachine.add('BASE', Base(), 
                               transitions={'check_elevator':'ELEVATOR_CHECK'},
                               remapping={'task_progress_in':'task_progress',
                                          'task_progress_out':'task_progress'})


        smach.StateMachine.add('ELEVATOR_CHECK', Elevator_Goal_Check(), 
                               transitions={'move_elevator':'ELEVATOR',
                                            'check_arm':'ARM_CHECK'},
                               remapping={'task_progress_in':'task_progress',
                                          'task_progress_out':'task_progress',
                                          'home_position_out':'home_position'})
        smach.StateMachine.add('ELEVATOR', Elevator(), 
                               transitions={'check_arm':'ARM_CHECK',
                                            'task_progress':'TASK_PROGRESS'},
                               remapping={'task_progress_in':'task_progress',
                                          'task_progress_out':'task_progress',
                                          'home_position_in':'home_position'})


        smach.StateMachine.add('ARM_CHECK', Arm_Goal_Check(), 
                               transitions={'move_arm':'ARM',
                                            'task_progress':'TASK_PROGRESS'},
                               remapping={'task_progress_in':'task_progress',
                                          'task_progress_out':'task_progress',
                                          'home_position_out':'home_position'})
        smach.StateMachine.add('ARM', Arm(), 
                               transitions={'task_progress':'TASK_PROGRESS'},
                               remapping={'task_progress_in':'task_progress',
                                          'task_progress_out':'task_progress',
                                          'home_position_in':'home_position'})

    # Create and start the introspection server
    # sis = smach_ros.IntrospectionServer('robot_state_machine', sm, '/SM_ROOT')
    # sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    # sis.stop()