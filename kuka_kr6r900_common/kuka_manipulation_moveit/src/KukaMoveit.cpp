#include "kuka_manipulation_moveit/KukaMoveit.hpp"

namespace rvt = rviz_visual_tools;

KukaMoveit::KukaMoveit(std::string manipulatorGroupName) :
    MANIPULATOR_GROUP(manipulatorGroupName),
    move_group(manipulatorGroupName)
{

    move_group.startStateMonitor();

    // Visual tools
    visual_tools = new moveit_visual_tools::MoveItVisualTools(move_group.getPlanningFrame());
    visual_tools->deleteAllMarkers();
    visual_tools->loadRemoteControl();

    // Getting Basic Information
    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

    std::vector<double> jv = move_group.getCurrentJointValues();
    std::vector<std::string> jn = move_group.getJointNames();
    for (size_t i = 0; i < jv.size(); ++i)
        ROS_INFO("Joint %s: %f", jn[i].c_str(), jv[i]);


    configure();
}

KukaMoveit::~KukaMoveit()
{}

void KukaMoveit::configure()
{
    /** Configure move group **/
    move_group.setPlanningTime(10);
    move_group.setNumPlanningAttempts(20);
    move_group.setGoalPositionTolerance(0.001);
    move_group.setPlannerId("RRTConnectkConfigDefault");
}

bool KukaMoveit::execute(bool showPath)
{
    if (showPath) {
        bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // Visualisation
        if (success) {
            Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
            const geometry_msgs::PoseStamped & p = move_group.getPoseTarget();
            ROS_INFO("Visualizing plan as trajectory line");

            visual_tools->publishAxisLabeled(p.pose, "pose");
            visual_tools->publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
            visual_tools->publishTrajectoryLine(plan.trajectory_,
                move_group.getRobotModel()->getJointModelGroup(MANIPULATOR_GROUP), rvt::GREEN);
            visual_tools->trigger();
            visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
        } else {
            ROS_ERROR("Plan construct FAILD");
            return false;
        }
    }

    if (move_group.move() != moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_ERROR("Plan to execute move function FAILD");
        return false;
    }
    return true;
}

bool KukaMoveit::move(const geometry_msgs::Pose & pose, bool showPath)
{
    move_group.setPoseTarget(pose);

    return execute(showPath);
}

bool KukaMoveit::move(const std::string name, bool showPath)
{
    move_group.setNamedTarget(name);

    return execute(showPath);
}