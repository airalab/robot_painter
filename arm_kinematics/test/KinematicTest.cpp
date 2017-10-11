#include "arm_kinematics/KR6ArmKinematics.h"
#include <iostream>

int main(int argc, const char *argv[])
{
    KR6ArmKinematics kinematics;

    JointValues jv;
    JointValues sol;
    Vector3d pos = kinematics.solveFK(jv);
    Vector3d pos1;
    Vector3d orient;
    Pose pose;
    pose.position = pos;
    pose.orientation = orient;
    std::vector<std::vector<double>> config = {{0, 1, 1}, {0, 1, -1}, {0, -1, 1}, {0, -1, -1}, 
        {M_PI, 1, 1}, {M_PI, 1, -1}, {M_PI, -1, 1}, {M_PI, -1, -1}};
                                             
    for (int i = 0; i < 8; ++i) {
        std::cout << "Configuration: " << config[i][0] << ", " << config[i][1] << ", " << config[i][2] << std::endl;
        std::cout << "=================================================" << std::endl;
        if (!kinematics.solveIK(pose, config[i], sol)) continue;

        std::cout << "Position: " << pos(0) << ", " << pos(1) << ", " << pos(2) << std::endl;
        std::cout << "Angles: " << sol(0) << ", " << sol(1) << ", " << sol(2) << ", " << sol(3) << ", " << sol(4) << ", " << sol(5) << std::endl;

        pos1 = kinematics.solveFK(sol);
        std::cout << "test Position: " << pos1(0) << ", " << pos1(1) << ", " << pos1(2) << std::endl << std::endl;
    }
    return 0;
}
