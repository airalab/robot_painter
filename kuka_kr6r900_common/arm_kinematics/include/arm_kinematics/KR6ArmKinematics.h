#ifndef KR6_ARM_KINEMATICS
#define KR6_ARM_KINEMATICS

#include "KinematicConstants.h"
#include "DataTypes.h"
#include <vector>

namespace kr6_arm_kinematics
{

class Kinematics {

    public:

        Kinematics(const std::vector<double> & minAngles, const std::vector<double> & maxAngles);
        ~Kinematics();

        // Forward kinematics
        Vector3d FK(const JointValues & jointValues);

        // Inverse kinematics for one configuration
        bool IK(const Pose & pose, const std::vector<double> & configuration, JointValues & solution);

        // Incerse kinematics for all possible configurations
        int getAllIKSolutions(const Pose & pose, std::vector<JointValues> & solutions);



    private:
        matrix::Dcm<double> calcRotMatix(const double phi, const double theta, const double psi);

        bool checkAngles(const JointValues & jointValues);

        JointValues minAng, maxAng;
};

}

#endif
