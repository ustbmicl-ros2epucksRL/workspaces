uint8 VICTIM=1
uint8 FIRE=2
uint8 BOX=3

std_msgs/Header header
uint8 object_type
int32 object_id
bool discovered
string detector_robot
geometry_msgs/PoseStamped precise_pose
geometry_msgs/PoseStamped goal_pose
bool has_goal_pose
float32 detection_radius
string scene_name
