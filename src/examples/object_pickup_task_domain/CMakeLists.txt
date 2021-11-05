cmake_minimum_required(VERSION 2.8.3)
project(rosplan_tiago_demo)

find_package(catkin REQUIRED COMPONENTS
        rospy
        roscpp
        std_msgs
        std_srvs
        diagnostic_msgs
        geometry_msgs
        move_base
        move_base_msgs
        rosplan_dispatch_msgs
        rosplan_knowledge_msgs
        rosplan_planning_system
        rosplan_interface_movebase
        actionlib
        actionlib_msgs
        tf2
        genmsg
        control_msgs
        tf
        )

find_package(OpenCV REQUIRED
        COMPONENTS
        opencv_highgui)

#roslaunch_add_file_check(launch)

add_definitions(-Wall -g -O0 -Wno-deprecated -static -Bstatic -std=gnu++0x)

catkin_package(CATKIN_DEPENDS actionlib_msgs)

include_directories(${catkin_INCLUDE_DIRS})

set(COMMON_LIB ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(set_rviz_initial_tiago_pose src/set_rviz_initial_tiago_pose.cpp)
target_link_libraries(
        set_rviz_initial_tiago_pose
        ${catkin_LIBRARIES})
add_dependencies(
        set_rviz_initial_tiago_pose
        ${catkin_EXPORTED_TARGETS})

add_executable(set_gazebo_tiago_cube_pose src/set_gazebo_tiago_cube_pose.cpp)
target_link_libraries(
        set_gazebo_tiago_cube_pose
        ${catkin_LIBRARIES})
add_dependencies(
        set_gazebo_tiago_cube_pose
        ${catkin_EXPORTED_TARGETS})

#add_executable(record_path_node src/record_robot_realtime_path.cpp)
#add_dependencies(record_path_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(record_path_node ${catkin_LIBRARIES})
#
#add_executable(calculate_path_node src/calculate_path_length.cpp)
#add_dependencies(calculate_path_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(calculate_path_node ${catkin_LIBRARIES})




