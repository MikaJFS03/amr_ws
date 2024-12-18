cmake_minimum_required(VERSION 3.8)
project(edabot_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

add_executable(friend_pose src/friend_pose.cpp)
ament_target_dependencies(friend_pose rclcpp geometry_msgs sensor_msgs)

add_executable(navigate_to_goal_action_client src/navigate_to_goal_action_client.cpp)
ament_target_dependencies(navigate_to_goal_action_client rclcpp std_msgs nav2_msgs rclcpp_action)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  friend_pose
  navigate_to_goal_action_client
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
