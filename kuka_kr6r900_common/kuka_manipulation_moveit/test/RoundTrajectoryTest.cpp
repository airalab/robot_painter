#include <ros/ros.h>

#include "kuka_manipulation_moveit/KukaMoveit.hpp"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "round_traj_execution");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    KukaMoveit manipulator("manipulator");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose center, pose;
    size_t pointsNum = 30;
    double h = 0.05;
    double angle = 0, angelStep = 2*M_PI/pointsNum, radius = 0.1;

    // Point 1
    center.orientation.w = 1.0;
    center.position.x = 0.6;
    center.position.y = 0.0;
    center.position.z = 0.4;

    waypoints.resize(pointsNum);
    pose.position.x = center.position.x;
    for (size_t i = 0; i < pointsNum; ++i) {

        pose.position.y = radius*cos(-angle) + center.position.y;
        pose.position.z = radius*sin(-angle) + center.position.z;
        std::cout << "p: (" << i << ") \t [" << pose.position.y << ", " << pose.position.z << "]" << std::endl;

        angle += angelStep;
        waypoints[i] = pose;

    }

    // Set slow motion
    manipulator.getMoveGroup()->setMaxVelocityScalingFactor(0.1);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    double fraction = manipulator.getMoveGroup()->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory
    );

    for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {

        std::cout << "(" << i << ")\t" << trajectory.joint_trajectory.points[i].time_from_start << " | "
            << trajectory.joint_trajectory.points[i].positions[0] << std::endl;

    }

    plan.trajectory_ = trajectory;
    manipulator.getMoveGroup()->execute(plan);

    return 1;
}