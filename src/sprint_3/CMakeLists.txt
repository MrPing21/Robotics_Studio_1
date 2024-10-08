cmake_minimum_required(VERSION 3.8)
project(sprint_3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Add the executable first
add_executable(cylinder_detection_node src/cylinder_detection_node.cpp)

# Link ROS2 packages
ament_target_dependencies(cylinder_detection_node 
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# Link libraries to the executable after defining it
target_link_libraries(cylinder_detection_node 
  ${OpenCV_LIBRARIES}
)

# Install the node executable
install(TARGETS
  cylinder_detection_node 
  DESTINATION lib/${PROJECT_NAME}
)
# Find Doxygen
find_package(Doxygen)

if(DOXYGEN_FOUND)
      configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)

    # Create a target for Doxygen documentation
    add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM)

endif(DOXYGEN_FOUND)

# Package
ament_package()


