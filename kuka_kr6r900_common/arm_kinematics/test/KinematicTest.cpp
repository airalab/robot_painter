#include "arm_kinematics/KR6ArmKinematics.h"
#include <iostream>

int main(int argc, const char *argv[])
{
    kr6_arm_kinematics::Kinematics kinematics(minJointAngles, maxJointAngles);
    std::vector<JointValues> solutions;

    // Set position
    Vector3d pos(0.647132, 0.210288, 0.663268);

    // Set orientation by quaternion
    matrix::Euler<double> ang(matrix::Quaternion<double> (1, 0, 0, 0));
    // Vector3d orient(ang.phi(), ang.theta(), ang.psi());
    Vector3d orient(0, 0, 0);
    ang.print();

    Pose pose;
    pose.position = pos;
    pose.orientation = orient;

    Vector3d pos1;
    JointValues sol;


    int result = kinematics.getAllIKSolutions(pose, solutions);
    if (result = 2) {
        std::cout << ("No solutions found") << std::endl;
    }

    for (int i = 0; i < solutions.size(); ++i) {

        sol = solutions[i];

        std::cout << "=================================================" << std::endl;

        std::cout << "Position: " << pos(0) << ", " << pos(1) << ", " << pos(2) << std::endl;
        std::cout << "Angles: " << sol(0) << ", " << sol(1) << ", " << sol(2) << ", " << sol(3) << ", " << sol(4) << ", " << sol(5) << std::endl;

        pos1 = kinematics.FK(sol);
        std::cout << "test Position: " << pos1(0) << ", " << pos1(1) << ", " << pos1(2) << std::endl << std::endl;
    }
    return 0;
}
