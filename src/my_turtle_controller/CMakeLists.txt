cmake_minimum_required(VERSION 3.8)
project(my_turtle_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

# Source files
add_executable(turtle_controller src/turtle_controller.cpp)

add_executable(spawn_turtle src/spawn_turtle.cpp)

# Link libraries
ament_target_dependencies(
  turtle_controller
  rclcpp
  turtlesim
  geometry_msgs
  example_interfaces
  my_robot_interfaces
)
ament_target_dependencies(
  spawn_turtle
  rclcpp
  turtlesim
  geometry_msgs
  example_interfaces
  my_robot_interfaces
)
# Install the executable
install(
  TARGETS turtle_controller spawn_turtle
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
