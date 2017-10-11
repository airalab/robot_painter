#include <ros/ros.h>
#include <string>
#include "arm_manipulation/Manipulator.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "move_arm");
    ros::NodeHandle nh;

    // Initialize manipulator
    std::string prefix = "joint_a";
    Manipulator manipulator(prefix, nh);
    manipulator.initArmTopics();

    Pose pose;
    double x = 0.525, y = 0, z = 0.89, alpha = 0, beta = 0, gamma = 0;
    std::vector<double> config = {0, 1, 1};
    std::string question;

    std::cout << "Do you wont move the arm? (y, n): "; std::cin >> question;
    while(nh.ok() && (question == "y" || question == "\n")) {
        std::cout << "Position: "; std::cin >> x >> y >> z;
        std::cout << "Orientation: "; std::cin >> alpha >> beta >> gamma;
        std::cout << "Configuration: "; std::cin >> config[0] >> config[1] >> config[2];

        pose.position(0) = x;
        pose.position(1) = y;
        pose.position(2) = z;
        pose.orientation(0) = alpha;
        pose.orientation(1) = beta;
        pose.orientation(2) = gamma;

        manipulator.moveArm(pose, config);
        std::cout << "Do you wont move the arm? (y, n): "; std::cin >> question;
    }
    return 0;
}
