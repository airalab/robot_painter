#include "arm_kinematics/KR6ArmKinematics.h"
#include <iostream>

using namespace kr6_arm_kinematics;

#define KINEMATIC_DEBUG true

Kinematics::Kinematics(const std::vector<double> & minAngles, const std::vector<double> & maxAngles)
{
    for (size_t i = 0; i < DOF; ++i) {
        minAng(i) = minAngles[i];
        maxAng(i) = maxAngles[i];
    }
}
Kinematics::~Kinematics()
{}

Vector3d Kinematics::FK(const JointValues & jointValues)
{

    Vector3d position;
    double q1 = jointValues(0);
    double q2 = jointValues(1);
    double q3 = jointValues(2);
    double q4 = jointValues(3);
    double q5 = jointValues(4);

    position(0) = d1*cos(q1) + d2*cos(q1)*cos(q2) + d3*sin(q2+q3)*cos(q1) + d4*cos(q1)*cos(q2+q3) -
        d5*((sin(q1)*sin(q4) + sin(q2+q3)*cos(q1)*cos(q4))*sin(q5) - cos(q1)*cos(q5)*cos(q2+q3));
    position(1) = - d1*sin(q1) - d2*sin(q1)*cos(q2) - d3*sin(q1)*sin(q2+q3) - d4*sin(q1)*cos(q2+q3)
        + d5*((sin(q1)*sin(q2+q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) - sin(q1)*cos(q5)*cos(q2+q3));
    position(2) = d0 - d2*sin(q2) + d3*cos(q2+q3) - d4*sin(q2+q3) - d5*(sin(q5)*cos(q4)*cos(q2+q3)
        + sin(q2+q3)*cos(q5));

    return position;
}


int Kinematics::getAllIKSolutions(const Pose & pose, std::vector<JointValues> & solutions)
{

    JointValues solution;
    std::vector<std::vector<double>> config = {{0, 1, 1}, {0, 1, -1}, {0, -1, 1}, {0, -1, -1},
    {M_PI, 1, 1}, {M_PI, 1, -1}, {M_PI, -1, 1}, {M_PI, -1, -1}};

    if (solutions.size() != 0)
        return 2; // Invalid initial size of solutions vector

    for (int i = 0; i < 8; ++i) {
        if (!IK(pose, config[i], solution)) continue;
        solutions.push_back(solution);
    }

    if (solutions.size() == 0)
        return 2; // No solution found

    return 1; // OK
}

bool Kinematics::IK(const Pose & pose, const std::vector<double> & configuration, JointValues & solution)
{

    if (configuration.size() != 3) {
        if (KINEMATIC_DEBUG)
            std::cerr << "Invalid configuration paramters, array size" << configuration.size() << std::endl;
        return false;
    }

    Vector3d pos = pose.position;
    Vector3d planePos;
    Vector3d orient = pose.orientation;
    matrix::Dcm<double> rotMatrix = calcRotMatix(orient(0), orient(1), orient(2));

    double d = 0, cosAng = 0, ang = 0, cosq5 = 0;
    double q1 = 0, q2 = 0, q3 = 0, q4 = 0, q5 = 0, q6 = 0;


    // Shift goal from link_6 to link_5
    pos -= d5*Vector3d(rotMatrix(0,0), rotMatrix(1,0), rotMatrix(2,0));
    // pos.print();

    q1 = atan2(-pos(1), pos(0)); // plus/minus pi
    if (configuration[0] != 0) {
        if (q1 > M_PI + minJointAngles[0]) q1 -= configuration[0];
        else if (q1 < maxJointAngles[0] - M_PI) q1 += configuration[0];
        else {
            if (KINEMATIC_DEBUG)
                std::cerr << "Solution with configuration: (" << configuration[0] << ", " << configuration[1] << ", " << configuration[2] << ") is NOT exist." << std::endl;
            return false;
        }
    }

    // pos.print();
    // planePos(0) = cos(q1)*pos(0) - sin(q1)*pos(1) - d1;
    // planePos(1) = sin(q1)*pos(0) + cos(q1)*pos(1);
    // planePos(2) = pos(2) - d0;
    planePos = calcRotMatix(0, 0, q1)*pos - Vector3d(d1, 0, d0);
    // planePos.print();


    d = sqrt(d4*d4 + d3*d3);
    cosAng = (planePos(0)*planePos(0) + planePos(2)*planePos(2) - d2*d2 - d*d)/(2*d2*d);

    if (cosAng > 1) {
        if (KINEMATIC_DEBUG)
            std::cerr << "Solution with configuration: (" << configuration[0] << ", " << configuration[1] << ", " << configuration[2] << ") is NOT exist. Cosine of angle > 1." << std::endl;
        return false;
    }

    ang = configuration[1]*atan2(sqrt(1 - cosAng*cosAng), cosAng); // plus/minus
    q3 = ang + atan(d3/d4);
    q2 = atan2(-planePos(2), planePos(0)) - atan2(d*sin(ang), d2 + d*cos(ang));


    // TODO check orientation solution
    // Rotation solution
    q4 = atan2(rotMatrix(0,0)*sin(q1) + rotMatrix(1,0)*cos(q1),
        rotMatrix(0,0)*sin(q2+q3)*cos(q1) - rotMatrix(1,0)*sin(q1)*sin(q2+q3) + rotMatrix(2, 0)*cos(q2+q3));

    cosq5 = rotMatrix(0,0)*cos(q1)*cos(q2+q3) - rotMatrix(1,0)*sin(q1)*cos(q2+q3) - rotMatrix(2, 0)*sin(q2+q3);
    q5 = configuration[2]*atan2(sqrt(1 - cosq5*cosq5), cosq5); // plus/minus

    q6 = atan2(rotMatrix(0,1)*cos(q1)*cos(q2 + q3) - rotMatrix(1,1)*sin(q1)*cos(q2 + q3) - rotMatrix(2,1)*sin(q2 + q3),
        rotMatrix(0,2)*cos(q1)*cos(q2 + q3) + rotMatrix(1,2)*sin(q1)*cos(q2 + q3) + rotMatrix(2,2)*sin(q2 + q3));

    solution(0) = q1; solution(1) = q2; solution(2) = q3; solution(3) = q4; solution(4) = q5; solution(5) = q6;

    bool valid = checkAngles(solution);
    if (valid == false) {
        if (KINEMATIC_DEBUG)
            std::cout << "Solution not valid. Anglse out of range. Configuration: " << configuration[0] << ", " << configuration[1] << ", " << configuration[2] << std::endl;
        return false;
    }

    Vector3d testPosition = FK(solution);
    Vector3d diffPosition = testPosition - pose.position;
    if (diffPosition.norm() > 0.001) {
        if (KINEMATIC_DEBUG)
            std::cout << "Solution not valid. Position is not equal to task. Configuration: " << configuration[0] << ", " << configuration[1] << ", " << configuration[2] << std::endl;
        return false;
    }

    return true;
}

matrix::Dcm<double> Kinematics::calcRotMatix(const double phi, const double theta, const double psi)
{
    return matrix::Dcm<double> (matrix::Euler<double> (phi, theta, psi));
}

bool Kinematics::checkAngles(const JointValues & jointValues)
{
    bool valid = true;
    for (size_t i = 0; i < DOF; ++i) {
        if (jointValues(i) < minAng(i) || jointValues(i) > maxAng(i)) {
            valid = false;
            break;
       }
    }
    return valid;
}