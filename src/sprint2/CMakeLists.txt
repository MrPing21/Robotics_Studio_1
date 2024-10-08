cmake_minimum_required(VERSION 3.8)
project(sprint2)


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Include the include directories for the project
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Add the executable for the localizer node
add_executable(sprint2 src/publisher_member_function.cpp)

# Link libraries to the executable
target_link_libraries(sprint2
  ${OpenCV_LIBRARIES}
)

# Link ROS2 packages
ament_target_dependencies(sprint2
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# Install the node executable
install(TARGETS
  sprint2
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
