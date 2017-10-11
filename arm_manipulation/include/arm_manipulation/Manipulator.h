#ifndef MANIUPATOR
#define MANIUPATOR

#include <ros/ros.h>
#include <string>

#include <brics_actuator/JointPositions.h>
#include <arm_kinematics/KR6ArmKinematics.h>


// Message generation
brics_actuator::JointPositions createArmPositionMsg(std::string prefix, JointValues jointAngles);

class Manipulator
{
    public:
        Manipulator(std::string prefix, ros::NodeHandle & nodeHandle);
        ~Manipulator();

        void initArmTopics();

        void moveArm(const JointValues & jointValues);
        void moveArm(const Pose & pose, const std::vector<double> & configuration);

    private:
        ros::NodeHandle nh;

        // Kinematic solver
        KR6ArmKinematics solver;

        // Publishers
        ros::Publisher armPublisher;

        std::string jointPrefix;

};

#endif
