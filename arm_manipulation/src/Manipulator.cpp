#include "arm_manipulation/Manipulator.h"

Manipulator::Manipulator(std::string prefix, ros::NodeHandle & nodeHandle)
    :nh(nodeHandle), jointPrefix(prefix)
{}

Manipulator::~Manipulator() {}


void Manipulator::initArmTopics()
{
    ROS_INFO_STREAM("[AM] Load topics ...");
    ROS_INFO_STREAM("[AM] Publisher: /kuka_arm/arm_controller/position_command" << "\n");
    armPublisher = nh.advertise<brics_actuator::JointPositions> ("/kuka_arm/arm_controller/position_command", 1);
    ros::Duration(1).sleep();
}

void Manipulator::moveArm(const JointValues & jointValues)
{
    if (!solver.checkAngles(jointValues)) {
        ROS_ERROR_STREAM("[AM] Angles invalid joint angles.");
        return;
    }

    brics_actuator::JointPositions jointPositions = createArmPositionMsg(jointPrefix, jointValues);
    
    ROS_INFO_STREAM("[AM] Move to pose.");
    armPublisher.publish(jointPositions);
}
void Manipulator::moveArm(const Pose & pose, const std::vector<double> & configuration)
{
    JointValues jointAngles;
    
    // All configurations for manipulator
    std::vector<std::vector<double>> config = {{0, 1, 1}, {0, 1, -1}, {0, -1, 1}, {0, -1, -1}, 
        {M_PI, 1, 1}, {M_PI, 1, -1}, {M_PI, -1, 1}, {M_PI, -1, -1}};

    if (solver.solveIK(pose, configuration, jointAngles)) {
        moveArm(jointAngles);
    }

    // If solution not found, find first avalible solution
    for (size_t i = 0; i < config.size(); ++i) {
        if (solver.solveIK(pose, config[i], jointAngles))
            moveArm(jointAngles);
    }
}

brics_actuator::JointPositions createArmPositionMsg(std::string prefix, JointValues jointAngles)
{
    brics_actuator::JointPositions jointPositions; 
    brics_actuator::JointValue jointValue;

    for (size_t i = 0; i < DOF; ++i) {
        jointValue.timeStamp = ros::Time::now();
        std::stringstream jointName;
        jointName << prefix << (i + 1);
        jointValue.joint_uri = jointName.str();
        jointValue.unit = "rad";
        jointValue.value = jointAngles(i); 
        jointPositions.positions.push_back(jointValue);
    }
    return jointPositions;
}
