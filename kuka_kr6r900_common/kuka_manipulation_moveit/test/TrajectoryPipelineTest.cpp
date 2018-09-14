#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <ros/package.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_tutorial");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh("~");

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

    // Visualization
    // ^^^^^^^^^^^^^
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    /* Remote control is an introspection tool that allows users to step through a high level script
     via buttons and keyboard shortcuts in RViz */
    visual_tools.loadRemoteControl();

    /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XLARGE);

    /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
    visual_tools.trigger();

    /* We can also use visual_tools to wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Pose Goal
    // ^^^^^^^^^
    // We will now create a motion plan request for the right arm of the Panda
    // specifying the desired pose of the end-effector as input.
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = 0.5;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.7;
    pose.pose.orientation.w = 1.0;

    // A tolerance of 0.01 m is specified in position
    // and 0.01 radians in orientation
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    // We will create the request as a constraint using a helper function available
    // from the
    // `kinematic_constraints`_
    // package.
    //
    // .. _kinematic_constraints:
    //     http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
    req.group_name = "manipulator";
    moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("tool0", pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);

    // Now, call the pipeline and check whether planning was successful.
    planning_pipeline->generatePlan(planning_scene, req, res);
    /* Check that the planning was successful */
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
    }

    // Visualize the result
    // ^^^^^^^^^^^^^^^^^^^^
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

    // Using a Planning Request Adapter
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // A planning request adapter allows us to specify a series of operations that
    // should happen either before planning takes place or after the planning
    // has been done on the resultant path

    /* First, set the state in the planning scene to the final state of the last plan */
    robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
    planning_scene->setCurrentState(response.trajectory_start);
    // const robot_model::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("manipulator");
    robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

    // Now, set one of the joints slightly outside its upper limit
    const robot_model::JointModel* joint_model = joint_model_group->getJointModel("joint_a3");
    const robot_model::JointModel::Bounds& joint_bounds = joint_model->getVariableBounds();
    std::vector<double> tmp_values(1, 0.0);
    tmp_values[0] = joint_bounds[0].min_position_ + 0.1;
    ROS_INFO_STREAM("tmp val: " << tmp_values[0]);
    robot_state.setJointPositions(joint_model, tmp_values);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal);

    // Call the planner again and visualize the trajectories
    planning_pipeline->generatePlan(planning_scene, req, res);
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
    /* Now you should see three planned trajectories in series*/
    display_publisher.publish(display_trajectory);

    /* Wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish the demo");


    // Trajectory parametization
    robot_trajectory::RobotTrajectory robotTraj = robot_trajectory::RobotTrajectory(*res.trajectory_);
    moveit_msgs::RobotTrajectory robotTrajMsg;

    double accelScalingFactor = 0.7;
    double velScalingFactor = 1.0;

    // Parabolic parametrization
    trajectory_processing::IterativeSplineParameterization isp;
    if (!isp.computeTimeStamps(robotTraj, velScalingFactor, accelScalingFactor))
        return 1;

    // Parabolic parametrization
    // trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // if (!iptp.computeTimeStamps(robotTraj))
        // return 1;
    robotTraj.getRobotTrajectoryMsg(robotTrajMsg);

    /* Write trajectory to file */
    trajectory_msgs::JointTrajectory printedTrajectory = robotTrajMsg.joint_trajectory;

    std::string dataProcessingPackagePath = ros::package::getPath("rsi_tests");
    std::stringstream csvfilePath;
    csvfilePath << dataProcessingPackagePath << "/data/data1.csv";
    ROS_INFO("File path: %s", csvfilePath.str().c_str());
    boost::filesystem::ofstream ofs{csvfilePath.str()};
    csvfilePath.str("");

    for (size_t i = 0; i < printedTrajectory.points.size(); ++i) {

        // Write to file
        ofs << printedTrajectory.points[i].time_from_start;
        for (size_t joint = 0; joint < 6; ++joint)
            ofs << ", " << printedTrajectory.points[i].positions[joint];
        ofs << "\n";

    }
    ofs.close();


    ROS_INFO("Done");

    return 0;
}