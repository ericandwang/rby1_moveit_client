cmake_minimum_required(VERSION 4.0.2)
project(rby1_moveit_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rby1-sdk REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# Include rby1-sdk headers
include_directories(~/rby1-sdk/include)

# Include Eigen directories
include_directories(
  ${EIGEN3_INCLUDE_DIRS}  # Add this
  /home/nvidia/rby1-sdk/include
  ${sensor_msgs_INCLUDE_DIRS}
)

# init base
add_executable(init_base src/init_base.cpp)
ament_target_dependencies(init_base rclcpp trajectory_msgs controller_manager_msgs)
install(TARGETS init_base DESTINATION lib/${PROJECT_NAME})

# simple base
add_executable(simple_base src/simple_base.cpp)
ament_target_dependencies(simple_base rclcpp trajectory_msgs sensor_msgs geometry_msgs)
install(TARGETS simple_base DESTINATION lib/${PROJECT_NAME})
target_link_libraries(simple_base rby1-sdk::rby1-sdk)

# simple moveit client
add_executable(moveit_simple_client src/moveit_simple_client.cpp)
ament_target_dependencies(moveit_simple_client rclcpp moveit_ros_planning_interface geometry_msgs)
install(TARGETS moveit_simple_client DESTINATION lib/${PROJECT_NAME})

# simple gripper
add_executable(simple_gripper src/simple_gripper.cpp)
ament_target_dependencies(simple_gripper rclcpp dynamixel_sdk)
install(TARGETS simple_gripper DESTINATION lib/${PROJECT_NAME})
target_link_libraries(simple_gripper rby1-sdk::rby1-sdk)

# moveit + gripper test
add_executable(moveit_gripper_test src/moveit_gripper_test.cpp)
ament_target_dependencies(moveit_gripper_test rclcpp dynamixel_sdk moveit_ros_planning_interface geometry_msgs)
install(TARGETS moveit_gripper_test DESTINATION lib/${PROJECT_NAME})
target_link_libraries(moveit_gripper_test rby1-sdk::rby1-sdk)

# moveit + gripper + base test
add_executable(moveit_gripper_base_test src/moveit_gripper_base_test.cpp)
ament_target_dependencies(moveit_gripper_base_test rclcpp dynamixel_sdk moveit_ros_planning_interface geometry_msgs trajectory_msgs sensor_msgs)
install(TARGETS moveit_gripper_base_test DESTINATION lib/${PROJECT_NAME})
target_link_libraries(moveit_gripper_base_test rby1-sdk::rby1-sdk)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
