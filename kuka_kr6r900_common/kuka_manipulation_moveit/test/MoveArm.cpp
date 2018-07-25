#include <ros/ros.h>

#include "kuka_manipulation_moveit/KukaMoveit.hpp"
#include <geometry_msgs/Pose.h>

const std::string topicName = "/kuka_arm/pose_command";
const int RATE = 10;
bool start = false;

geometry_msgs::Pose pose;

const double x_min = 0.1;
const double x_max = 0.8;
const double y_min = -0.4;
const double y_max = 0.4;
const double z_min = 0.2;
const double z_max = 0.6;

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

    /* Set joint constraints */

    moveit_msgs::Constraints constraints;
    constraints.joint_constraints.resize(1);
    constraints.joint_constraints[0].joint_name = "joint_a1";
    constraints.joint_constraints[0].position = 0.0;
    constraints.joint_constraints[0].tolerance_above = 0.6;
    constraints.joint_constraints[0].tolerance_below = 0.6;
    constraints.joint_constraints[0].weight = 1.0;
    manipulator.getMoveGroup()->setPathConstraints(constraints);


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
