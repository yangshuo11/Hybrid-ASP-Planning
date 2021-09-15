cmake_minimum_required(VERSION 2.8.3)
project(rosplan_interface_movebase)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  diagnostic_msgs
  geometry_msgs
  move_base_msgs
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  rosplan_planning_system
  actionlib
  tf2
  genmsg
  nav_msgs
  message_generation
)

add_service_files(
        FILES
        DetectImage.srv
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rpmovebase src/RPMoveBase.cpp)
target_link_libraries(rpmovebase ${catkin_LIBRARIES})

add_executable(set_rviz_robot_pose src/set_rviz_robot_pose.cpp)
target_link_libraries(set_rviz_robot_pose ${catkin_LIBRARIES})
add_dependencies(set_rviz_robot_pose ${catkin_EXPORTED_TARGETS})

add_executable(set_gazebo_cube_pose src/set_gazebo_cube_pose.cpp)
target_link_libraries(set_gazebo_cube_pose ${catkin_LIBRARIES})
add_dependencies(set_gazebo_cube_pose ${catkin_EXPORTED_TARGETS})

add_executable(record_path_node src/record_robot_realtime_path.cpp)
add_dependencies(record_path_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_path_node ${catkin_LIBRARIES})

add_executable(calculate_path_node src/calculate_path_length.cpp)
add_dependencies(calculate_path_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(calculate_path_node ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS rpmovebase
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
