#include <ros/ros.h>

#include <arm_manipulation/Manipulator.h>
#include <kuka_cv/Colour.h>
#include <kuka_cv/SetMode.h>
#include <kuka_cv/RequestPalette.h>
#include <kuka_cv/RequestCanvas.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "camera_test");
    ros::NodeHandle nh;

    // Service client
    ros::ServiceClient cameraModeClient = nh.serviceClient<kuka_cv::SetMode>("/SetPaleteDetectionMode");
    ros::ServiceClient paletteClient = nh.serviceClient<kuka_cv::RequestPalette>("/request_palette");
    ros::ServiceClient canvasClient = nh.serviceClient<kuka_cv::RequestCanvas>("/request_canvas");

    // Initialize manipulator
    std::string prefix = "joint_a";
    Manipulator manipulator(prefix, nh);
    manipulator.initArmTopics();

    std::string question = "";
    std::cout << "Start? (y,n): "; std::cin >> question;
    if (question != "y")
        return 0;

    // Initialization
    // Default manipulator configuration
    std::vector<double> config = {0, 1, 1};

    // Palette colours
    std::vector<kuka_cv::Colour> colours;

    // Some poses
    // Calc by experiment
    Pose detectPalettePose;
    detectPalettePose.position(0) = 0.1;
    detectPalettePose.position(1) = 0.4;
    detectPalettePose.position(2) = 0.85;
    detectPalettePose.orientation(0) = 0.0;
    detectPalettePose.orientation(1) = 0.0;
    detectPalettePose.orientation(2) = 1.5708;

    Pose detectCanvasPose = detectPalettePose;
    detectCanvasPose.position(1) = -0.4;
    detectCanvasPose.orientation(2) = -1.5708;

    kuka_cv::SetMode cameraSrv;
    kuka_cv::RequestPalette paletteInfo;
    kuka_cv::RequestCanvas canvasInfo;

    JointValues initiatlJV;
    initiatlJV(1) = -M_PI/2;
    initiatlJV(2) = M_PI/2;

    ROS_INFO_STREAM("[LTP] Move to initial pose");
    manipulator.moveArm(initiatlJV);

    /// Palette detection and checking
    ROS_INFO_STREAM("[LTP] Move to Palette");
    manipulator.moveArm(detectPalettePose, config);
    ros::Duration(0.1).sleep();

    cameraSrv.request.mode = 1;
    ROS_INFO_STREAM("[LTP] Detect colours positions on palette: ");
    if (cameraModeClient.call(cameraSrv)) {
        ROS_INFO_STREAM("\t Successful");
    }

    ROS_INFO_STREAM("[LTP] Receive palette message.");
    do {
        if (paletteClient.call(paletteInfo)) {
            ROS_INFO_STREAM("\t Successful");
        }
        colours = paletteInfo.response.colours;
        ROS_WARN_STREAM("[LTP] Receive colours array size = 0");
    } while (colours.size() == 0);

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

    /// Canvas detection
    ROS_INFO_STREAM("[LTP] Move to Canvas");
    manipulator.moveArm(detectCanvasPose, config);
    ros::Duration(0.1).sleep();

    cameraSrv.request.mode = 2;
    ROS_INFO_STREAM("[LTP] Detect canvas transformation and dimensions: ");
    if (cameraModeClient.call(cameraSrv)) {
        ROS_INFO_STREAM("\t Successful");
    }

    ROS_INFO_STREAM("[LTP] Receive canvas message: ");
    do {
        if (canvasClient.call(canvasInfo)) {
            ROS_INFO_STREAM("\t Successful");
        }
    
        ROS_WARN_STREAM("[LTP] Receive wrong canvas info");
    } while (canvasInfo.response.width == 0);
    ROS_INFO_STREAM("\t Successful");
    return 0;
}