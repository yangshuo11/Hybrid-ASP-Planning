//
// Created by ys on 7/2/21.
//
#include <ros/ros.h>
#include <iostream>
#include <fstream>
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calculate_path_node");
    double data_x, data_y;
    double a,b;
    double total_path_length = 0.0;
    std::vector<double> robot_poses_x;
    std::vector<double> robot_poses_y;
    ifstream myfile("trial_30_tiago_path.txt",ios_base::in);
    if (myfile.fail()) {
        std::cout << "file does not exist" << std::endl;
    } else {
        std::cout << "read data (x,y) from previously recorded file" << std::endl;
        while (myfile >> a >> b) {
            data_x = a;
            data_y = b;
            robot_poses_x.push_back(data_x);
            robot_poses_y.push_back(data_y);
//            std::cout << "Line: (x,y) = " << "(" << data_x << "," << data_y << ")" << std::endl;
        }
    }
    for (int i = 0; i < robot_poses_x.size() - 1; i++) {
        double temp = sqrt(pow(robot_poses_x[i+1] - robot_poses_x[i],2) +
                           pow(robot_poses_y[i+1] - robot_poses_y[i],2));
        total_path_length += temp;
    }
    std::cout << "The total path length: " << total_path_length << std::endl;
}

