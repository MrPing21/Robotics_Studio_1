cmake_minimum_required(VERSION 3.8)
project(week2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(week2 src/subscriber_member_function.cpp)
ament_target_dependencies(week2 rclcpp std_msgs sensor_msgs)


target_link_libraries(week2
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)



install(TARGETS
  week2
  DESTINATION lib/${PROJECT_NAME})

ament_package()
