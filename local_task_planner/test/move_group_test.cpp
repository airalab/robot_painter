#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char ** argv)
{

	ros::init(argc, argv, "move_group_test");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Planning group
	static const std::string PLANNING_GROUP = "manipulator";
	std::vector<double> jv; // Joint values
	std::vector<std::string> jn; // Joint names

	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	const robot_state::JointModelGroup* joint_model_group =
		move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	/** Configure move group **/
	move_group.setPlanningTime(10);
	move_group.setNumPlanningAttempts(20);
	move_group.setGoalPositionTolerance(0.001);
	move_group.setPlannerId("RRTConnectkConfigDefault");

	// Visualisation
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
	visual_tools.deleteAllMarkers();

	visual_tools.loadRemoteControl();

	Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
	text_pose.translation().z() = 1.75;
	visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

	visual_tools.trigger();

	// Getting Basic Information
	ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

	// We can also print the name of the end-effector link for this group.
	ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

	move_group.getCurrentState()->copyJointGroupPositions(joint_model_group, jv);
	jn = joint_model_group->getJointModelNames();
	for (size_t i = 0; i < jv.size(); ++i)
		ROS_INFO("Joint %s: %f", jn[i].c_str(), jv[i]);

	visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

	geometry_msgs::Pose pose;
	pose.orientation.w = 1.0;
	pose.position.x = 0.5;
	pose.position.y = 0.0;
	pose.position.z = 0.3;
	move_group.setPoseTarget(pose);
	move_group.move();

	// Create plan
	// moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	// bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	// // Visualisation
	// ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
	// visual_tools.publishAxisLabeled(pose, "pose1");
	// visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	// visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group, rvt::GREEN);
	// visual_tools.trigger();
	// visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

	pose.orientation.w = 1.0;
	pose.position.x = 0.5;
	pose.position.y = 0.0;
	pose.position.z = 0.2;
	move_group.setPoseTarget(pose);
	move_group.move();

	pose.orientation.w = 1.0;
	pose.position.x = 0.4;
	pose.position.y = 0.0;
	pose.position.z = 0.2;
	move_group.setPoseTarget(pose);
	move_group.move();

	pose.orientation.w = 0.0;
	pose.orientation.z = 1.0;
	pose.position.x = 0.15;
	pose.position.y = 0.4;
	pose.position.z = 0.85;
	move_group.setPoseTarget(pose);
	move_group.move();

	moveit_msgs::MotionPlanRequest req;
	move_group.constructMotionPlanRequest(req);

	ROS_INFO("Group name: %s", req.group_name.c_str());
	ROS_INFO("Num planning attempts: %d", req.num_planning_attempts);
	ROS_INFO("Allowed planning time: %f", req.allowed_planning_time);
	ROS_INFO("Planner ID: %s", req.planner_id.c_str());
	ROS_INFO_STREAM("Goal count: " << req.goal_constraints.size());

	// moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	// bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	// // Visualisation
	// ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
	// visual_tools.publishAxisLabeled(pose, "pose2");
	// visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	// visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group, rvt::GREEN);
	// visual_tools.trigger();
	// visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

	move_group.move();

	return 0;
}