# SensorData.msg
# Message to represent sensor data from the robot

# Timestamp
builtin_interfaces/Time stamp

# Sensor type (e.g., "camera", "lidar", "imu", "force_torque")
string sensor_type

# Sensor name
string sensor_name

# Sensor data (flexible array to accommodate different sensor types)
float64[] data

# Associated frame
string frame_id

# Data quality metrics
float64 confidence
float64 timestamp_offset