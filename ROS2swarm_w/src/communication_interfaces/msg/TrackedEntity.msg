uint8 ROBOT=0
uint8 OBJECT=1
uint8 TARGET=2

string name
uint8 entity_type
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
builtin_interfaces/Time source_stamp
bool pose_valid
bool twist_valid
float32 confidence
