add_library(logical_camera_plugin SHARED 
  logical_camera_plugin.cpp logical_camera_plugin.h
)

target_link_libraries(logical_camera_plugin 
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
)

