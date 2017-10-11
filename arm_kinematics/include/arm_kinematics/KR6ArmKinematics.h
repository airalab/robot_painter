#ifndef KR6_ARM_KINEMATICS
#define KR6_ARM_KINEMATICS

#include "KinematicConstants.h"
#include "DataTypes.h"
#include <vector>

class KR6ArmKinematics {
    public:
        Vector3d solveFK(const JointValues & jointValues);
        bool solveIK(const Pose & pose, const std::vector<double> & configuration, JointValues & solution);
        bool checkAngles(const JointValues & jointValues);
        
    private:
        matrix::Matrix<double, 3, 3> calcRotMatix(const double alpha, const double beta, const double gamma);

};

#endif
