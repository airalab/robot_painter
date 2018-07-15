#include <kr6_kinematics_plugin/KR6KinematicsPlugin.h>
#include <ros/ros.h>

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "test_plugin");
	ros::NodeHandle nh;

	kr6_kinematics_plugin::KR6KinematicsPlugin kinematicPlugin();
}