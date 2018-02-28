#ifndef MANIUPATOR
#define MANIUPATOR

#include <ros/ros.h>
#include <string>

#include <sensor_msgs/JointState.h>
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

        bool moveArm(JointValues jointValues);
        bool moveArm(const Pose & pose, const std::vector<double> & configuration);

    private:
        void stateCallback(const sensor_msgs::JointStatePtr & msg);

        ros::NodeHandle nh;

        // Kinematic solver
        KR6ArmKinematics solver;

        // Publishers
        ros::Publisher armPublisher;
        ros::Subscriber jointStateSubscriber;

        std::string jointPrefix;
        JointValues jointState;

};

#endif
