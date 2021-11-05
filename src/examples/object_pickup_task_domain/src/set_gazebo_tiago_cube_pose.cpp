//
// Created by ys on 6/22/21.
//
#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <thread>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "set_gazebo_tiago_cube_pose");
    ros::NodeHandle nh;
    ros::service::waitForService("/gazebo/set_model_state");
    std::this_thread::sleep_for (std::chrono::milliseconds(3000));
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState robot_state, cube_state;
    // robot original position
    double robot_origin_pos_x = -0.00750165003447;
    double robot_origin_pos_y = -0.000984486956456;
    double robot_origin_pos_z = -0.00164366811696;
    double robot_origin_ori_x = 0.00287421658918;
    double robot_origin_ori_y = 0.000639633808716;
    double robot_origin_ori_z = -0.00233601301513;
    double robot_origin_ori_w = 0.99999293637;

    // cube original position
    double cube_origin_pos_x = 0.68296782871;
    double cube_origin_pos_y = -0.0248320438212;
    double cube_origin_pos_z = 0.86500059614;
    double cube_origin_ori_x = 2.7208837159e-05;
    double cube_origin_ori_y = -1.94391030002e-07;
    double cube_origin_ori_z = 0.0011311982138;
    double cube_origin_ori_w = 0.999999359825;

    // cube possible position_1 (rotation observable)
    double cube_possible_pos1_x = 0.688561428651;
    double cube_possible_pos1_y = -0.328466197246;
    double cube_possible_pos1_z = 0.864995433657;
    double cube_possible_ori1_x = -5.1287743234e-07;
    double cube_possible_ori1_y = -8.96182992401e-08;
    double cube_possible_ori1_z = 0.00552427030297;
    double cube_possible_ori1_w = 0.999984741102;
    // tiago fetch cup in position_1
    double tiago_pos1_x = -0.000682282949142;
    double tiago_pos1_y = -0.000282008842064;
    double tiago_pos1_z = -0.00159361736912;
    double tiago_ori1_x = 0.0026834635744;
    double tiago_ori1_y = 0.000728823013411;
    double tiago_ori1_z = -0.00146693650443;
    double tiago_ori1_w = 0.999995057957;
    // cube possible position_2 (transition observable)
    double cube_possible_pos2_x = 2.06041231903;
    double cube_possible_pos2_y = -0.0506190874171;
    double cube_possible_pos2_z = 0.864996766867;
    double cube_possible_ori2_x = -4.35107753001e-05;
    double cube_possible_ori2_y = -1.1611970726e-07;
    double cube_possible_ori2_z = 0.00155186245747;
    double cube_possible_ori2_w = 0.999998794914;
    // tiago fetch cup in position_2
    double tiago_pos2_x = 2.61212333491;
    double tiago_pos2_y = -0.0022436156019;
    double tiago_pos2_z = -0.00157654947018;
    double tiago_ori2_x = -0.000489712341125;
    double tiago_ori2_y = 0.00102482512403;
    double tiago_ori2_z = 0.999919334242;
    double tiago_ori2_w = 0.0126504910579;

    // tiago stand in the middle of the table
    double tiago_middle_pos_x = 1.4153478771;
    double tiago_middle_pos_y = -0.794749303837;
    double tiago_middle_pos_z = -0.00159685552558;
    double tiago_middle_ori_x = 0.000723181790881;
    double tiago_middle_ori_y = 0.00125458390756;
    double tiago_middle_ori_z = 0.00125458390756;
    double tiago_middle_ori_w = 0.694986659936;

    cube_state.request.model_state.model_name = "aruco_cube_0"; //
    cube_state.request.model_state.pose.position.x = cube_possible_pos2_x;
    cube_state.request.model_state.pose.position.y = cube_possible_pos2_y;
    cube_state.request.model_state.pose.position.z = cube_possible_pos2_z;
    cube_state.request.model_state.pose.orientation.x = cube_possible_ori2_x;
    cube_state.request.model_state.pose.orientation.y = cube_possible_ori2_y;
    cube_state.request.model_state.pose.orientation.z = cube_possible_ori2_z;
    cube_state.request.model_state.pose.orientation.w = cube_possible_ori2_w;
    cube_state.request.model_state.twist.linear.x = 0.0;
    cube_state.request.model_state.twist.linear.y = 0.0;
    cube_state.request.model_state.twist.linear.x = 0.0;
    cube_state.request.model_state.twist.angular.x = 0.0;
    cube_state.request.model_state.twist.angular.y = 0.0;
    cube_state.request.model_state.twist.angular.z = 0.0;
    cube_state.request.model_state.reference_frame = "world";
    if (client.call(cube_state))
    {
        ROS_INFO("Successfully set the cube pose in gazebo world");
    }
    else
    {
        ROS_INFO("Failed to set the cube pose");
    }

//    robot_state.request.model_state.model_name = "tiago";
//    robot_state.request.model_state.pose.position.x = tiago_pos2_x;
//    robot_state.request.model_state.pose.position.y = tiago_pos2_y;
//    robot_state.request.model_state.pose.position.z = tiago_pos2_z;
//    robot_state.request.model_state.pose.orientation.x = tiago_ori2_x;
//    robot_state.request.model_state.pose.orientation.y = tiago_ori2_y;
//    robot_state.request.model_state.pose.orientation.z = tiago_ori2_z;
//    robot_state.request.model_state.pose.orientation.w = tiago_ori2_w;
//    robot_state.request.model_state.twist.linear.x = 0.0;
//    robot_state.request.model_state.twist.linear.y = 0.0;
//    robot_state.request.model_state.twist.linear.x = 0.0;
//    robot_state.request.model_state.twist.angular.x = 0.0;
//    robot_state.request.model_state.twist.angular.y = 0.0;
//    robot_state.request.model_state.twist.angular.z = 0.0;
//    robot_state.request.model_state.reference_frame = "world";
//    if (client.call(robot_state))
//    {
//        ROS_INFO("Successfully set the tiago pose in gazebo world");
//    }
//    else
//    {
//        ROS_INFO("Failed to set the tiago pose");
//    }

}
