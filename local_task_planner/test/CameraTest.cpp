#include <ros/ros.h>

#include <arm_manipulation/Manipulator.h>
#include <kuka_cv/Palette.h>
#include <kuka_cv/Colour.h>

std::vector<kuka_cv::Colour> colours;

void paletteCallback(const kuka_cv::Palette & msg)
{
    if (msg.colours.size() > 0) {
        colours = msg.colours;
    } else {
        ROS_ERROR_STREAM("[LTP] Empty message.");
        return;
    }

}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "camera_test");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber paletteSubscriber = nh.subscribe("Palette", 10, paletteCallback);

    // Initialize manipulator
    std::string prefix = "joint_a";
    Manipulator manipulator(prefix, nh);
    manipulator.initArmTopics();

    std::string question = "";
    std::cout << "Start? (y,n): "; std::cin >> question;
    if (question != "y")
        return 0;

    JointValues initiatlJV;
    initiatlJV(1) = -M_PI/2;
    initiatlJV(2) = M_PI/2;

    ROS_INFO_STREAM("[LTP] Receive palette message.");
    while (colours.size() == 0)
        ros::spinOnce();

    ROS_INFO_STREAM("[LTP] Move to initial pose");
    manipulator.moveArm(initiatlJV);
    ros::Duration(5).sleep();

    ROS_INFO_STREAM("[LTP] Move to colours on palette");
    Pose p;
    std::vector<double> config = {0, 1, 1};
    for (size_t i = 0; i < colours.size(); ++i) {
        ROS_INFO_STREAM("[LTP] Number: " << (i + 1) << " --------------");
        ROS_INFO_STREAM("[LTP] Colour: [" << colours[i].bgr[0] 
                                  << ", " << colours[i].bgr[1] 
                                  << ", " << colours[i].bgr[2] << "]");

        p.orientation(2) = 1.5708;
        p.position(0) = colours[i].position[0];
        p.position(1) = colours[i].position[1] - 0.025;    // 0.025 - width/2 of holder
        p.position(2) = 0.2;                               // 0.2 - deistance between brush and link 6

        manipulator.moveArm(p, config);
        ros::Duration(5).sleep();
    }


    return 0;
}