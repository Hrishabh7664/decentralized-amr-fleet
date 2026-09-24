std_msgs/Header header
string robot_id
geometry_msgs/Pose pose
geometry_msgs/Twist twist
float32 battery_percentage
bool is_charging
string current_state # IDLE, NAVIGATING, RESOLVING_CONFLICT, CHARGING, BLOCKED
