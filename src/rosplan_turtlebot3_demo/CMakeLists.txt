cmake_minimum_required(VERSION 2.8.3)
project(rosplan_turtlebot3_demo)

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

        )





