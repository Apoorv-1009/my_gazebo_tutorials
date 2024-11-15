add_library(walker_controller
  walker_controller.cpp
)

target_include_directories(walker_controller PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

ament_target_dependencies(walker_controller
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
)