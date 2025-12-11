# RobotState.msg
# Message to represent the state of a robot

# Timestamp
builtin_interfaces/Time stamp

# Joint states
string[] joint_names
float64[] joint_positions
float64[] joint_velocities
float64[] joint_efforts

# Robot status
string status
bool is_enabled
bool is_moving