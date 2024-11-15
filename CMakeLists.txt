cmake_minimum_required(VERSION 3.8)
project(walker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# add subdirectories
add_subdirectory(libs)

include_directories(libs)

# add executables
add_executable(turtlebot_controller src/turtlebot_controller.cpp)
ament_target_dependencies(turtlebot_controller rclcpp std_msgs sensor_msgs geometry_msgs)

target_link_libraries(turtlebot_controller walker_controller)

# install executables
install(TARGETS
  turtlebot_controller
  DESTINATION lib/${PROJECT_NAME}
)

# install directories
install(DIRECTORY 
launch
worlds
libs
DESTINATION share/${PROJECT_NAME})

ament_package()
