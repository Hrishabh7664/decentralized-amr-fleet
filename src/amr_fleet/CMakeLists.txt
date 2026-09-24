cmake_minimum_required(VERSION 3.8)
project(amr_fleet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/RobotIntent.msg"
  "msg/TaskBid.msg"
  "msg/Conflict.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files, configs, and world files
install(DIRECTORY
  launch
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install executable Python scripts
install(PROGRAMS
  amr_fleet/state_publisher.py
  amr_fleet/intent_publisher.py
  amr_fleet/global_planner.py
  amr_fleet/local_planner_orca.py
  amr_fleet/conflict_resolver.py
  amr_fleet/task_allocator.py
  amr_fleet/battery_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
