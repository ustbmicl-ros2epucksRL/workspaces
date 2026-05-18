std_msgs/Header header

uint8 VICTIM=1
uint8 FIRE=2
uint8 BOX=3

uint8 NEW=1
uint8 DISPATCHED=2
uint8 EN_ROUTE=3
uint8 ACTING=4
uint8 DONE=5
uint8 FAILED=6

int32 task_id
uint8 task_type
uint8 status

int32 object_id
string detector_robot
string scene_name

geometry_msgs/PoseStamped precise_pose
geometry_msgs/PoseStamped staging_pose
geometry_msgs/PoseStamped goal_pose
bool has_goal_pose

float32 hold_seconds
string behavior_type
string[] assigned_robots
