//
// Created by ys on 7/2/21.
//
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <math.h>
#include <fstream>
#include <iostream>

using namespace std;

ros::Subscriber model_sub;
geometry_msgs::Pose base_pose_;
std::vector<geometry_msgs::Pose> pose_array;
double path_length = 0;

void model_state_callback(const gazebo_msgs::ModelStatesConstPtr &msg)
{

    ofstream robot_pose_file, robot_pose_x, robot_pose_y;
    robot_pose_file.open("visit_wp2_trial_30_path.txt", ios::app);
    robot_pose_x.open("visit_wp2_trial_30_x.txt", ios::app);
    robot_pose_y.open("visit_wp2_trial_30_y.txt", ios::app);

    int modelCount = msg->name.size();
    for (int i = 0; i < modelCount; i++)
    {
        if (msg->name[i] == "turtlebot3_waffle")
        {
//            std::cout << "output to file" << std::endl;
            geometry_msgs::Pose pose = msg->pose[i];
            robot_pose_file << pose.position.x << " " << pose.position.y << "\n";
            robot_pose_x << pose.position.x << "\n";
            robot_pose_y << pose.position.y << "\n";
        }

    }
    robot_pose_file.close();
    robot_pose_x.close();
    robot_pose_y.close();
}

//void base_pose_cb(const nav_msgs::Odometry::ConstPtr& odom)
//{
//    ofstream robot_pose_file, robot_pose_x, robot_pose_y;
//    robot_pose_file.open("Hybrid_approach_target_wp4_plan_8.txt", ios::app);
//    robot_pose_x.open("Hybrid_approach_target_wp4_plan_x_8.txt", ios::app);
//    robot_pose_y.open("Hybrid_approach_target_wp4_plan_y_8.txt", ios::app);
//    base_pose_ = odom->pose.pose;
//    robot_pose_file << base_pose_.position.x << " " << base_pose_.position.y << "\n";
//    robot_pose_x << base_pose_.position.x << "\n";
//    robot_pose_y << base_pose_.position.y << "\n";
//    robot_pose_file.close();
//    robot_pose_x.close();
//    robot_pose_y.close();
////    pose_array.push_back(base_pose_);
////    for (int i = 1; i < pose_array.size(); i ++) {
////        double temp = sqrt(pow(pose_array[i].position.x - pose_array[i-1].position.x, 2) +
////                           pow(pose_array[i].position.y - pose_array[i-1].position.y, 2));
////        path_length += temp;
////    }
////    std::cout << "total path length = " << path_length << std::endl;
//}


int main(int argc,char **argv) {

    ros::init(argc, argv, "record_path_node");
    ros::NodeHandle nh("~");
//    base_pose_sub_ = nh.subscribe("/odom", 1, &base_pose_cb);
    model_sub = nh.subscribe("/gazebo/model_states", 1, &model_state_callback);
    ros::spin();
}


