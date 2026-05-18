std_msgs/Header header

uint8 IDLE=1
uint8 NAVIGATING=2
uint8 ACTING=3
uint8 DONE=4
uint8 FAILED=5

uint8 state
int32 active_task_id
uint8 active_task_type
string behavior_type
string detail
geometry_msgs/PoseStamped formation_center
geometry_msgs/PoseStamped target_pose
float32 hold_remaining
string[] assigned_robots
