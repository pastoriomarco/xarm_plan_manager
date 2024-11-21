cmake_minimum_required(VERSION 3.8)
project(xarm_plan_manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Enable additional compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)  # Added tf2_ros as it's used in the code

# Include directories for header files
include_directories(include)

# Declare the executable with both source files
add_executable(plan_manager_node
  src/plan_manager_node.cpp
  src/plan_manager.cpp
)

# Specify libraries to link against
ament_target_dependencies(plan_manager_node
  rclcpp
  geometry_msgs
  xarm_msgs
  moveit
  moveit_ros_planning_interface
  moveit_msgs
  tf2_ros
)

# Install the executable to the appropriate location
install(TARGETS plan_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory (if present)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Testing configuration (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # Skip certain linters if necessary
  # Uncomment the following lines if you want to skip specific linters
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize the package
ament_package()
