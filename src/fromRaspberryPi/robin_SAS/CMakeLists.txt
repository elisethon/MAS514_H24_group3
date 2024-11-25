cmake_minimum_required(VERSION 3.5)
project(robin_SAS)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Specify message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelSpeeds.msg"
)

# Install Python packages
install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(
  PROGRAMS
  # Add your Python executables here
  joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
