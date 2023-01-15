int8 task_type
int8 TASK_PICK = 0
int8 TASK_HOME = 1
int8 TASK_SWAP = 2
int8 TASK_SHUTDOWN = 3
geometry_msgs/PoseStamped base_orientation
float64 elevator_height
geometry_msgs/PoseStamped arm_orientation
float64[] arm_joint_angles
bool gripper_closed