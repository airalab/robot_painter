#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

const std::string RVIZ_MARKER_TOPIC = "/rviz_visual_tools";
rviz_visual_tools::colors color = rviz_visual_tools::colors::RED;

int main(int argc, char ** argv)
{

	ros::init(argc, argv, "move_group_test");
	ros::NodeHandle nh;
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
	robot_model::RobotModelConstPtr robot = move_group.getRobotModel();
	robot_state::RobotState currentState(robot);

	/** Planning scene initialization **/
	planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot));
	planning_scene->setCurrentState(currentState);

    planning_pipeline::PlanningPipelinePtr planning_pipeline(
        new planning_pipeline::PlanningPipeline(robot, nh, "planning_plugin", "request_adapters"));


	// Visualisation
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("base_link", RVIZ_MARKER_TOPIC, robot);
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

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    moveit_msgs::MotionPlanResponse response;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = 0.4;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.2;
    pose.pose.orientation.w = 1.0;
    req.allowed_planning_time = 10;

    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    req.group_name = PLANNING_GROUP;
    moveit_msgs::Constraints pose_goal =
        kinematic_constraints::constructGoalConstraints("link_6", pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);

    planning_pipeline->generatePlan(planning_scene, req, res);
    /* Check that the planning was successful */
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
    }

	ros::Publisher display_publisher =
	    nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    /* Visualize the trajectory */
    ROS_INFO("Visualizing the trajectory");
    res.getMessage(response);

 //    display_trajectory.trajectory_start = response.trajectory_start;
 //    display_trajectory.trajectory.push_back(response.trajectory);
 //    display_publisher.publish(display_trajectory);
	// visual_tools.trigger();
	// visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    /* Fill plan */
    my_plan.start_state_ = response.trajectory_start;
    my_plan.trajectory_ = response.trajectory;
    my_plan.planning_time_ = response.planning_time;
	move_group.execute(my_plan);

	robot_state::RobotState & robot_state = planning_scene->getCurrentStateNonConst();
    planning_scene->setCurrentState(response.trajectory_start);
    robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

    pose.pose.position.x = 0.4;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.1;
    pose.pose.orientation.w = 1.0;
    moveit_msgs::Constraints pose_goal2 =
        kinematic_constraints::constructGoalConstraints("link_6", pose, tolerance_pose, tolerance_angle);

    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal2);

    planning_pipeline->generatePlan(planning_scene, req, res);
    /* Check that the planning was successful */
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      return 0;
    }

    /* Visualize the trajectory */
    ROS_INFO("Visualizing the trajectory");
    res.getMessage(response);

    // display_trajectory.trajectory_start = response.trajectory_start;
    // display_trajectory.trajectory.push_back(response.trajectory);
    // display_publisher.publish(display_trajectory);
    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    /* Fill plan */
 //    my_plan.start_state_ = response.trajectory_start;
 //    my_plan.trajectory_ = response.trajectory;
 //    my_plan.planning_time_ = response.planning_time;
	// move_group.execute(my_plan);

	// Visualisation
	// ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
	// visual_tools.publishAxisLabeled(pose.pose, "pose1");
	// visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	// visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group, rvt::RED);
	// visual_tools.trigger();
	// visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

	return 1;

}