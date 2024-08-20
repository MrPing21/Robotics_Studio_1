cmake_minimum_required(VERSION 3.8)
project(lab1)

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


add_executable(labone src/subscriber_member_function.cpp)
ament_target_dependencies(labone rclcpp std_msgs sensor_msgs cv_bridge OpenCV)


target_link_libraries(labone
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
)



install(TARGETS
  labone
  DESTINATION lib/${PROJECT_NAME})

ament_package()
