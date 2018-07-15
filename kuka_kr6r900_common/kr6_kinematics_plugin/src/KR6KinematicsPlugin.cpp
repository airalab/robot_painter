#include <kr6_kinematics_plugin/KR6KinematicsPlugin.h>
#include <kr6_kinematics_plugin/configuration_comparator.h>

#include <pluginlib/class_list_macros.h>
// #include <class_loader/class_loader.hpp>


// register KR6Kinematics as a KinematicsBase implementation
PLUGINLIB_EXPORT_CLASS(kr6_kinematics_plugin::KR6KinematicsPlugin, kinematics::KinematicsBase)
// CLASS_LOADER_REGISTER_CLASS(kr6_kinematics_plugin::KR6KinematicsPlugin, kinematics::KinematicsBase)

using namespace kr6_kinematics_plugin;

namespace kr6_kinematics_plugin
{

/**
*  @brief Default constructor
*/
KR6KinematicsPlugin::KR6KinematicsPlugin()
{}

KR6KinematicsPlugin::~KR6KinematicsPlugin()
{}

bool KR6KinematicsPlugin::initialize(const std::string& robot_description,
                const std::string& group_name,
                const std::string& base_frame,
                const std::string& tip_frame,
                double search_discretization)
{
    setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);


    urdf::Model robot_model;
    if (!robot_model.initParam(robot_description)) return false;

    std::vector<double> lower_limits;
    std::vector<double> upper_limits;
    if (!extractKinematicData(robot_model, base_frame, tip_frame,
            joint_names_, link_names_, lower_limits, upper_limits)) {
        return false;
    }

    // Validate that the correct number of joints has been extracted
    if (joint_names_.size() != DOF) return false;

    kinematics.reset(new kr6_arm_kinematics::Kinematics(lower_limits, upper_limits));

    return true;
}


bool KR6KinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                    const std::vector<double> &ik_seed_state,
                                    std::vector<double> &solution,
                                    moveit_msgs::MoveItErrorCodes &error_code,
                                    const kinematics::KinematicsQueryOptions &options) const
{

    // Check if the initialize function has already been called successfully
    if (!kinematics) return false;

    // Validate that there is one seed value per joint
    if (ik_seed_state.size() != DOF) return false;

    // Convert message
    Pose targetPose = poseMsgToMatrix(ik_pose);

    std::vector<JointValues> solutions;
    int res = kinematics->getAllIKSolutions(targetPose, solutions);

    if (res == 0) {
        error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
        return false;
    }

    JointValues initialAngles = configurationMsgToJv(ik_seed_state);
    solution = selectIKSolution(solutions, initialAngles);

    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
}

bool KR6KinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        std::vector<double> &solution,
                        moveit_msgs::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options) const
{

    IKCallbackFn solution_callback = 0;
    std::vector<double> consistency_limits;

    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
                            solution, solution_callback, error_code);

}

bool KR6KinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        const std::vector<double> &consistency_limits,
                        std::vector<double> &solution,
                        moveit_msgs::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options) const
{

    IKCallbackFn solution_callback = 0;

    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
                            solution, solution_callback, error_code);

}

bool KR6KinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        std::vector<double> &solution,
                        const IKCallbackFn &solution_callback,
                        moveit_msgs::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options) const
{

    std::vector<double> consistency_limits;

    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
                            solution, solution_callback, error_code);

}

bool KR6KinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        const std::vector<double> &consistency_limits,
                        std::vector<double> &solution,
                        const IKCallbackFn &solution_callback,
                        moveit_msgs::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options) const
{

    return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);

}

bool KR6KinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                        const std::vector<double> &joint_angles,
                        std::vector<geometry_msgs::Pose> &poses) const
{

    // TODO use FK function
    return false;

}

const std::vector<std::string> & KR6KinematicsPlugin::getJointNames() const
{
    return joint_names_;
}


const std::vector<std::string> & KR6KinematicsPlugin::getLinkNames() const
{
    return link_names_;
}

std::vector<double> KR6KinematicsPlugin::selectIKSolution(const std::vector<JointValues> & solutions, const JointValues & initialState) const
{
    std::vector<std::vector<double>> solutionsVector(solutions.size());
    for (size_t i = 0; i < solutions.size(); ++i)
        solutionsVector[i] = configurationJvToMsg(solutions[i]);
    std::vector<double> initAngles = configurationJvToMsg(initialState);

    ConfigurationComparator<double> comp(initAngles);
    std::sort(solutionsVector.begin(), solutionsVector.end(), comp);

    return solutionsVector[0];
}

bool KR6KinematicsPlugin::extractKinematicData(const urdf::Model &robot_model,
                        const std::string &base_frame,
                        const std::string &tip_frame,
                        std::vector<std::string> &joint_names,
                        std::vector<std::string> &link_names,
                        std::vector<double> &lower_limits,
                        std::vector<double> &upper_limits) const
{
    urdf::LinkConstSharedPtr link = robot_model.getLink(tip_frame);

    while ((link) && (link->name != base_frame)) {
        link_names.push_back(link->name);
        urdf::JointConstSharedPtr joint = link->parent_joint;

        // Don't consider invalid, unknown or fixed joints
        if ((!joint) || (joint->type == urdf::Joint::UNKNOWN)
                || (joint->type == urdf::Joint::FIXED)) {
            // Continue with the next link in the kinematic chain
            link = link->getParent();

            continue;
        }

        joint_names.push_back(joint->name);

        // Extract the joint limits
        if (joint->type != urdf::Joint::CONTINUOUS) {
            if (joint->safety) {
                lower_limits.push_back(joint->safety->soft_lower_limit);
                upper_limits.push_back(joint->safety->soft_upper_limit);
            } else {
                lower_limits.push_back(joint->limits->lower);
                upper_limits.push_back(joint->limits->upper);
            }
        } else {
            lower_limits.push_back(-M_PI);
            upper_limits.push_back( M_PI);
        }

        // Continue with the next link in the kinematic chain
        link = link->getParent();
    }

    // The kinematic chain ended and the base frame was not found
    if (!link) return false;

    // The data has been extracted from the tip to the base, but it is required
    // the other way round
    std::reverse(link_names.begin(), link_names.end());
    std::reverse(joint_names.begin(), joint_names.end());
    std::reverse(lower_limits.begin(), lower_limits.end());
    std::reverse(upper_limits.begin(), upper_limits.end());

    return true;
}

Pose KR6KinematicsPlugin::poseMsgToMatrix(const geometry_msgs::Pose & p) const
{

    Pose pose;
    matrix::Euler<double> angles (
        matrix::Quaternion<double> (p.orientation.x, p.orientation.y,
                                    p.orientation.z, p.orientation.w)
    );

    pose.position(0) = p.position.x;
    pose.position(1) = p.position.y;
    pose.position(2) = p.position.z;
    pose.orientation(0) = angles.phi();
    pose.orientation(1) = angles.theta();
    pose.orientation(2) = angles.psi();

    return pose;
}

std::vector<double> KR6KinematicsPlugin::configurationJvToMsg(const JointValues & jv) const
{
    std::vector<double> msg(DOF);
    for (size_t i = 0; i < DOF; ++i) {
        msg[i] = jv(i);
    }

    return msg;
}


JointValues KR6KinematicsPlugin::configurationMsgToJv(const std::vector<double> msg) const
{
    JointValues jv;
    for (size_t i = 0; i < DOF; ++i) {
        jv(i) = msg[i];
    }

    return jv;
}

} // namespace