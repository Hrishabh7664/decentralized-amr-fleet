std_msgs/Header header
string robot_id
string task_id
geometry_msgs/Point current_goal
geometry_msgs/PoseStamped[] planned_path
float32 estimated_arrival_time # seconds remaining
float32 priority_score
bool has_token
string reserved_corridor_id
