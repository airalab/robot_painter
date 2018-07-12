#ifndef KUKA_MOVEIT
#define KUKA_MOVEIT

/* Initialize moveIt */
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

class KukaMoveit
{
    public:
        KukaMoveit(std::string manipulatorGroupName);
        ~KukaMoveit();

        bool move(const geometry_msgs::Pose & pose, bool showPath = true);
        bool move(const std::string name, bool showPath = true);

        inline moveit::planning_interface::MoveGroupInterface * getMoveGroup()
        {
            return &move_group;
        }

    private:

        void configure();
        bool execute(bool showPath);

        // MoveGroup interface
        moveit::planning_interface::MoveGroupInterface move_group;

        // Planning scene for creation an ostacles
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // Name of manipulator kinematic group
        const std::string MANIPULATOR_GROUP;

        // Visual tools for visualization of trajectory
        moveit_visual_tools::MoveItVisualTools * visual_tools;

        // Manipulator trajectory
        moveit::planning_interface::MoveGroupInterface::Plan plan;

};

#endif