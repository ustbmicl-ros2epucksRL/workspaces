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
string planning_state
float32 longitudinal_scale
float32 lateral_scale
float32 rms_slot_error
float32 max_slot_error
float32 minimum_clearance
bool motion_enabled
geometry_msgs/PoseStamped center_reference
geometry_msgs/PoseStamped[] slot_targets
