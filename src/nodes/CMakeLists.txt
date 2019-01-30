add_executable(pose_broadcaster_node
  pose_broadcaster_node.cpp
)

target_link_libraries(pose_broadcaster_node
  ${catkin_LIBRARIES}
)

