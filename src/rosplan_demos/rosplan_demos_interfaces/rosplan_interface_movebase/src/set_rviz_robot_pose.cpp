//
// Created by ys on 6/22/21.
//
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "math.h"
#define PI 3.1415926

int main(int argc, char **argv)
{
    ros::init(argc, argv, "initilise_robot_pose_in_rviz");
    ros::NodeHandle nh;
    ros::Publisher initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
    ros::Rate loop_rate(1);

    int count = 0;
    double pos_x = -6.56593701238;
    double pos_y = 3.32484189691;
    double pos_z = -0.00100739914401;

    double ang_x = -2.12306585454e-06;
    double ang_y = 0.00158964984592;
    double ang_z = 0.000290214543259;
    double ang_w = 0.999998694391;

    while(ros::ok() && count <5)
    {
        count = count + 1;
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = pos_x;
        pose_msg.pose.pose.position.y = pos_y;
        pose_msg.pose.pose.position.z = pos_z;
        pose_msg.pose.pose.orientation.x = ang_x;
        pose_msg.pose.pose.orientation.y = ang_y;
        pose_msg.pose.pose.orientation.z = ang_z;
        pose_msg.pose.pose.orientation.w = ang_w;

        initial_pose_pub.publish(pose_msg);
        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;

}
