cmake_minimum_required(VERSION 3.8)
project(week5)

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
find_package(geometry_msgs REQUIRED)


add_executable(week5 src/publisher_member_function.cpp)
ament_target_dependencies(week5 rclcpp std_msgs sensor_msgs cv_bridge OpenCV geometry_msgs)


target_link_libraries(week5
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
)



install(TARGETS
  week5
  DESTINATION lib/${PROJECT_NAME})

ament_package()