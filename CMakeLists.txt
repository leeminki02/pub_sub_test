cmake_minimum_required(VERSION 3.8)
project(pub_sub_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(do_both src/read_and_write.cpp)
ament_target_dependencies(do_both rclcpp std_msgs)

install(TARGETS
  talker
  listener
  do_both
  DESTINATION lib/${PROJECT_NAME})

ament_package()