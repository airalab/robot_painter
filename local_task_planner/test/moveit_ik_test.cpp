#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    /** Initialize **/
    std::vector<double> joint_values;
    const std::string PLANNING_SCENE_SERVICE = "get_planning_scene";

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_ptr =
        std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    planning_scene_monitor_ptr->requestPlanningSceneState(PLANNING_SCENE_SERVICE);
    planning_scene_monitor::LockedPlanningSceneRW ps(planning_scene_monitor_ptr);
    ps->getCurrentStateNonConst().update();

    robot_state::RobotState current_state = ps->getCurrentState();
    robot_model::RobotModelConstPtr robot_model = ps->getRobotModel();

    const robot_state::JointModelGroup* joint_model_group = current_state.getJointModelGroup("manipulator");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
    current_state.copyJointGroupPositions("manipulator", joint_values);
    for(std::size_t i = 0; i < joint_names.size(); ++i) {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        // ROS_INFO("Joint %s: %f", joint_names[i].c_str(), *(jv + i));
    }

    /* Start path planning */
    planning_pipeline::PlanningPipelinePtr planning_pipeline(
        new planning_pipeline::PlanningPipeline(robot_model, nh, "planning_plugin", "request_adapters"));
    planning_scene::PlanningScenePtr planning_scene(planning_scene_monitor_ptr->getPlanningScene());

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();


    /* Remote control is an introspection tool that allows users to step through a high level script
       via buttons and keyboard shortcuts in RViz */
    visual_tools.loadRemoteControl();
    ros::Duration(2).sleep();

    /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XLARGE);

    /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
    visual_tools.trigger();

    ros::Duration(2).sleep();
    /* We can also use visual_tools to wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = 0.4;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.2;
    pose.pose.orientation.w = 1.0;
    req.allowed_planning_time = 10;

    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    req.group_name = "manipulator";
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
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);

    /* Wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // /* First, set the state in the planning scene to the final state of the last plan */
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
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);

    display_publisher.publish(display_trajectory);

    /* Wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    planning_scene->setCurrentState(response.trajectory_start);
    robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

    pose.pose.position.x = 0.5;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.1;
    pose.pose.orientation.w = 1.0;
    moveit_msgs::Constraints pose_goal3 =
        kinematic_constraints::constructGoalConstraints("link_6", pose, tolerance_pose, tolerance_angle);

    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal3);

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
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);

    display_publisher.publish(display_trajectory);

    /* Wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    return 0;
}