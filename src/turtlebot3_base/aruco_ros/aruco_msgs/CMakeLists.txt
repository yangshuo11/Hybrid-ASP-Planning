cmake_minimum_required(VERSION 2.8.3)
project(aruco_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
        genmsg
        diagnostic_msgs
        roscpp
        )

add_message_files( FILES Marker.msg MarkerArray.msg )

add_service_files(
        FILES CheckActionEffect.srv
)

generate_messages( DEPENDENCIES
        std_msgs
        geometry_msgs
        diagnostic_msgs
        )

catkin_package(
  CATKIN_DEPENDS
         message_generation message_runtime diagnostic_msgs geometry_msgs roscpp std_msgs genmsg
)
