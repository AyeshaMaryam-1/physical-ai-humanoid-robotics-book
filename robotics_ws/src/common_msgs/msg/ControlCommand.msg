# ControlCommand.msg
# Message to represent a control command for the robot

# Timestamp
builtin_interfaces/Time stamp

# Command type (e.g., "position", "velocity", "effort", "trajectory")
string command_type

# Joint commands
string[] joint_names
float64[] joint_commands

# Cartesian command (if applicable)
string frame_id
float64[] cartesian_position  # x, y, z
float64[] cartesian_orientation  # x, y, z, w (quaternion)

# Command parameters
float64 duration
float64 max_velocity
float64 max_acceleration