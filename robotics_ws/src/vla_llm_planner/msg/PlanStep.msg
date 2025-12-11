string action_type  # "navigation", "manipulation", "perception", "communication", etc.
string action_name   # "move_to", "pick_object", "detect_person", "speak", etc.
string[] action_params  # Additional parameters for the action
geometry_msgs/Pose target_pose
builtin_interfaces/Time expected_duration
string[] preconditions  # Conditions that must be true before executing
string[] effects        # Effects that will be true after executing
float32 success_probability  # Estimated probability of successful execution