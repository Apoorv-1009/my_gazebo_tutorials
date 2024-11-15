add_library(walker_controller
  walker_controller.cpp
)

target_include_directories(walker_controller PUBLIC
)


ament_target_dependencies(walker_controller
  "geometry_msgs"
)