#include <ros/ros.h>

#include "kuka_manipulation_moveit/KukaMoveit.hpp"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "paat_action");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    KukaMoveit manipulator("manipulator");

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.5;
    pose.position.y = 0.0;
    pose.position.z = 0.3;
    manipulator.move(pose);

    return 1;
}