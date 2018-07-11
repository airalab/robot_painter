#include <ros/ros.h>

#include "kuka_manipulation_moveit/KukaMoveit.hpp"
#include <geometry_msgs/Pose.h>

const std::string topicName = "/kuka_arm/pose_command";
const int RATE = 10;
bool start = false;

geometry_msgs::Pose pose;

void poseCallback(const geometry_msgs::Pose::ConstPtr & msg)
{
    pose.position = msg->position;
    pose.orientation = msg->orientation;
    start = true;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "paat_action");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO_STREAM("Subscribe to \t" << topicName);
    ros::Subscriber poseSubscriber = nh.subscribe(topicName, 10, poseCallback);

    KukaMoveit manipulator("manipulator");

    while (nh.ok()) {

        if (start) {
            ROS_INFO("Receive point: [%f, %f, %f | %f, %f, %f, %f]",
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

            if (!manipulator.move(pose))
                return 2;
            start = false;
        }

    }

    return 1;
}