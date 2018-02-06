#include "arm_kinematics/KR6ArmKinematics.h"
#include <iostream>
#define KINEMATIC_DEBUG false

Vector3d KR6ArmKinematics::solveFK(const JointValues & jointValues) {
 
    Vector3d position;
    double q1 = jointValues(0); 
    double q2 = jointValues(1);
    double q3 = jointValues(2);
    double q4 = jointValues(3);
    double q5 = jointValues(4);
    double q6 = jointValues(5);

    position(0) = d1*cos(q1) + d2*cos(q1)*cos(q2) + d3*sin(q2+q3)*cos(q1) + d4*cos(q1)*cos(q2+q3) - 
        d5*((sin(q1)*sin(q4) + sin(q2+q3)*cos(q1)*cos(q4))*sin(q5) - cos(q1)*cos(q5)*cos(q2+q3)); 
    position(1) = - d1*sin(q1) - d2*sin(q1)*cos(q2) - d3*sin(q1)*sin(q2+q3) - d4*sin(q1)*cos(q2+q3) 
        + d5*((sin(q1)*sin(q2+q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) - sin(q1)*cos(q5)*cos(q2+q3));
    position(2) = d0 - d2*sin(q2) + d3*cos(q2+q3) - d4*sin(q2+q3) - d5*(sin(q5)*cos(q4)*cos(q2+q3) 
        + sin(q2+q3)*cos(q5));
   
    return position;
}

bool KR6ArmKinematics::solveIK(const Pose & pose, const std::vector<double> & configuration, JointValues & solution) {

    if (configuration.size() != 3) {
        if (KINEMATIC_DEBUG)
            std::cerr << "Invalid configuration paramters, array size" << configuration.size() << std::endl;
        return false;
    }

    Vector3d pos = pose.position;
    Vector3d planePos;
    Vector3d orient = pose.orientation;
    matrix::Matrix<double, 3, 3> rotMatrix = calcRotMatix(orient(0), orient(1), orient(2));

    double d = 0, cosAng = 0, ang = 0, cosq5 = 0;
    double q1 = 0, q2 = 0, q3 = 0, q4 = 0, q5 = 0, q6 = 0;

    pos(0) -= d5*rotMatrix(0,0);
    pos(1) -= d5*rotMatrix(1,0);
    pos(2) -= d5*rotMatrix(2,0);

    // std::cout << "pos: (" << pos(0) << ", " << pos(1) << ")" << std::endl;

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

    planePos(0) = cos(q1)*pos(0) - sin(q1)*pos(1) - d1;
    planePos(1) = sin(q1)*pos(0) + cos(q1)*pos(1);
    planePos(2) = pos(2) - d0;


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

    Vector3d testPosition = solveFK(solution);
    Vector3d diffPosition = testPosition - pose.position;
    if (diffPosition.norm() > 0.001) {
        if (KINEMATIC_DEBUG)
            std::cout << "Solution not valid. Position is not equal to task. Configuration: " << configuration[0] << ", " << configuration[1] << ", " << configuration[2] << std::endl;
        return false;
    }

    return true;
}

matrix::Matrix<double, 3, 3> KR6ArmKinematics::calcRotMatix(const double alpha, const double beta, const double gamma) {

    matrix::Matrix<double, 3, 3> orientation;
    orientation(0,0) = cos(beta)*cos(gamma);
    orientation(0,1) = sin(alpha)*sin(beta)*cos(gamma) - sin(gamma)*cos(alpha);
    orientation(0,2) = sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma);
    orientation(1,0) = sin(gamma)*cos(beta);
    orientation(1,1) = sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma);
    orientation(1,2) = -sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha);
    orientation(2,0) = -sin(beta);
    orientation(2,1) = sin(alpha)*cos(beta);
    orientation(2,2) = cos(alpha)*cos(beta);

    // std::cout << "Rot Matrix: {" << orientation(0,0) << ", " << orientation(0,1) << ", " << orientation(0, 2) << "}" << std::endl;
    // std::cout << "Rot Matrix: {" << orientation(1,0) << ", " << orientation(1,1) << ", " << orientation(1, 2) << "}" << std::endl;
    // std::cout << "Rot Matrix: {" << orientation(2,0) << ", " << orientation(2,1) << ", " << orientation(2, 2) << "}" << std::endl;

    return orientation;
}

bool KR6ArmKinematics::checkAngles(const JointValues & jointValues)
{
    bool valid = true;
    for (size_t i = 0; i < DOF; ++i) {
        if (jointValues(i) < minJointAngles[i] || jointValues(i) > maxJointAngles[i]) {
            valid = false;
            break;
       }
    }
    return valid;
}
