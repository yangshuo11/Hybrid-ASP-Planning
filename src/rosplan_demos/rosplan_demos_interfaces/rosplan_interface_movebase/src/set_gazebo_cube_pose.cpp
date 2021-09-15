//
// Created by ys on 6/22/21.
//
#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <thread>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_gazebo_cube_pose");
    ros::NodeHandle nh;
    ros::service::waitForService("/gazebo/set_model_state");
    std::this_thread::sleep_for (std::chrono::milliseconds(3000));
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState robot_state, aruco_2_state;

    // aruco_2_possible_changed_position
    double aruco_2_possible_pos1_x = -4.14162969324;
    double aruco_2_possible_pos1_y = 1.74015235923;
    double aruco_2_possible_pos1_z = 0.0988293954324;
    double aruco_2_possible_rot1_x = 6.14379073547e-10;
    double aruco_2_possible_rot1_y = -2.19527721946e-05;
    double aruco_2_possible_rot1_z = -1.44316754541e-064;
    double aruco_2_possible_rot1_w = 0.999999999758;


    aruco_2_state.request.model_state.model_name = "aruco_cube_582"; //
    aruco_2_state.request.model_state.pose.position.x = aruco_2_possible_pos1_x;
    aruco_2_state.request.model_state.pose.position.y = aruco_2_possible_pos1_y;
    aruco_2_state.request.model_state.pose.position.z = aruco_2_possible_pos1_z;
    aruco_2_state.request.model_state.pose.orientation.x = aruco_2_possible_rot1_x;
    aruco_2_state.request.model_state.pose.orientation.y = aruco_2_possible_rot1_y;
    aruco_2_state.request.model_state.pose.orientation.z = aruco_2_possible_rot1_z;
    aruco_2_state.request.model_state.pose.orientation.w = aruco_2_possible_rot1_w;
    aruco_2_state.request.model_state.twist.linear.x = 0.0;
    aruco_2_state.request.model_state.twist.linear.y = 0.0;
    aruco_2_state.request.model_state.twist.linear.x = 0.0;
    aruco_2_state.request.model_state.twist.angular.x = 0.0;
    aruco_2_state.request.model_state.twist.angular.y = 0.0;
    aruco_2_state.request.model_state.twist.angular.z = 0.0;
    aruco_2_state.request.model_state.reference_frame = "world";
    if (client.call(aruco_2_state))
    {
        ROS_INFO("Successfully set the aruco_2_changed_position in gazebo world");
    }
    else
    {
        ROS_INFO("Failed to set the aruco_2_changed_position");
    }

}
