cmake_minimum_required(VERSION 3.8)
project(common_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Create message directory
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msg)

# Define message files
set(msg_files
  "msg/RobotState.msg"
  "msg/ControlCommand.msg"
  "msg/SensorData.msg"
)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

ament_package()