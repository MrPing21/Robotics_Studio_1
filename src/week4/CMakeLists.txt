cmake_minimum_required(VERSION 3.8)
project(week4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(week4 src/week4.cpp)
ament_target_dependencies(week4 rclcpp std_msgs sensor_msgs cv_bridge OpenCV nav_msgs tf2 tf2_geometry_msgs)  


target_link_libraries(week4
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
)



install(TARGETS
  week4
  DESTINATION lib/${PROJECT_NAME})

ament_package()
