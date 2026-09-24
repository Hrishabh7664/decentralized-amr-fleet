std_msgs/Header header
string conflict_id
string initiating_robot_id
string conflicting_robot_id
string conflict_type # VERTEX, EDGE, DEADLOCK, CORRIDOR
geometry_msgs/Point conflict_location
float32 initiating_priority
float32 conflicting_priority
string resolution_action # PROCEED, YIELD, REPLAN, TOKEN_ACQUIRED, TOKEN_WAIT
bool resolved
