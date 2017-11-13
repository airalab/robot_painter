#include <ros/ros.h>

#include <arm_manipulation/Manipulator.h>
#include <kuka_cv/Palette.h>
#include <kuka_cv/Colour.h>
#include <kuka_cv/SetMode.h>

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

    // Service client
    ros::ServiceClient cameraModeClient = nh.serviceClient<kuka_cv::SetMode>("/SetPaleteDetectionMode");

    // Initialize manipulator
    std::string prefix = "joint_a";
    Manipulator manipulator(prefix, nh);
    manipulator.initArmTopics();

    std::string question = "";
    std::cout << "Start? (y,n): "; std::cin >> question;
    if (question != "y")
        return 0;

    Pose detectPose;    // Calc by experiment
    detectPose.position(0) = 0.1;
    detectPose.position(1) = 0.4;
    detectPose.position(2) = 0.85;
    detectPose.orientation(0) = 0.0;
    detectPose.orientation(1) = 0.0;
    detectPose.orientation(2) = 1.5708;
    std::vector<double> config = {0, 1, 1};

    JointValues initiatlJV;
    initiatlJV(1) = -M_PI/2;
    initiatlJV(2) = M_PI/2;

    ROS_INFO_STREAM("[LTP] Move to initial pose");
    manipulator.moveArm(initiatlJV);

    ROS_INFO_STREAM("[LTP] Move to detect pose");
    manipulator.moveArm(detectPose, config);

    kuka_cv::SetMode cameraSrv;
    cameraSrv.request.mode = 1;         // Measuring mode of camera
    ROS_INFO_STREAM("[LTP] Turn on camera and detect palette: ");
    if (cameraModeClient.call(cameraSrv)) {
        ROS_INFO_STREAM("Successful");
    }

    ROS_INFO_STREAM("[LTP] Receive palette message.");
    while (colours.size() == 0)
        ros::spinOnce();

    ROS_INFO_STREAM("[LTP] Move to colours on palette");
    Pose p;
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
    }


    return 0;
}