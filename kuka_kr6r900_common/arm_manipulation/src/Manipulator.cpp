#include "arm_manipulation/Manipulator.h"
#define MANIPULATION_DEBUG false 

Manipulator::Manipulator(std::string prefix, ros::NodeHandle & nodeHandle)
    :nh(nodeHandle), jointPrefix(prefix)
{}

Manipulator::~Manipulator() {}


void Manipulator::initArmTopics()
{
    ROS_INFO_STREAM("[AM] Load topics ...");
    ROS_INFO_STREAM("[AM] Publisher: /kuka_arm/arm_controller/position_command" << "\n");
    armPublisher = nh.advertise<brics_actuator::JointPositions> ("/kuka_arm/arm_controller/position_command", 1);
    jointStateSubscriber = nh.subscribe("/joint_states", 10, &Manipulator::stateCallback, this);
    ros::Duration(1).sleep();
}

bool Manipulator::moveArm(const JointValues & jointValues)
{
    if (!solver.checkAngles(jointValues)) {
        ROS_ERROR_STREAM("[AM] Angles invalid joint angles.");
        return false;
    }

    brics_actuator::JointPositions jointPositions = createArmPositionMsg(jointPrefix, jointValues);
    
    if (MANIPULATION_DEBUG)
        ROS_INFO_STREAM("[AM] Move to pose.");

    armPublisher.publish(jointPositions);

    JointValues diff;
    JointValues prevState;
    bool check;
    do {
        prevState = jointState;
        do {
            ros::spinOnce();
            for (size_t i = 0; i < DOF; ++i) {
                check = check && (jointState(i) == prevState(i));
            }
        } while(check && nh.ok());
        diff = jointState - jointValues;
    } while(diff.norm() > 0.01 && nh.ok());

    return true;
}
bool Manipulator::moveArm(const Pose & pose, const std::vector<double> & configuration)
{
    JointValues jointAngles;
    
    // All configurations for manipulator
    std::vector<std::vector<double>> config = {{0, 1, 1}, {0, 1, -1}, {0, -1, 1}, {0, -1, -1}, 
        {M_PI, 1, 1}, {M_PI, 1, -1}, {M_PI, -1, 1}, {M_PI, -1, -1}};

    if (solver.solveIK(pose, configuration, jointAngles)) {
        return moveArm(jointAngles);
    }

    // If solution not found, find first avalible solution
    for (size_t i = 0; i < config.size(); ++i) {
        if (solver.solveIK(pose, config[i], jointAngles)) {
            return moveArm(jointAngles);
        }
    }

    ROS_ERROR_STREAM("[AM] Solution for pose (" 
        << pose.position(0) << ", "
        << pose.position(1) << ", " 
        << pose.position(2) << ") is NOT FOND");
}

void Manipulator::stateCallback(const sensor_msgs::JointStatePtr & msg) {
    // std::stringstream message;
    // message << "Angles (";
    for (size_t i = 0; i < DOF; ++i) {
        jointState(i) = msg->position[i];
        // message << msg->position[i] << ", ";
    }
    // ROS_INFO_STREAM(message.str());
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

