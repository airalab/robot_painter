#include <kuka_cv/PainterCV.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "cv_test");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    int freq = 10; // in hz (image updating frequency)

    // Configure camera work
    // Work modes::
    // 1 |  from Image      (set imagePath)
    // 2 |  from Web Camera (set deviceNumber)
    // 3 |  from Realsense  (nothing)
    // 4 |  from Topic      (set topicName)

    CameraWorkingInfo info;
    // Config for image;
    // info.deviceNumber = 0;
    // info.imagePath = "/home/senex/Tkests/RealsenseCV/SavedImages/Image.jpg";

    // Calculation
    cv::Point_<double> colorPoint1_px(47, -124);
    cv::Point_<double> colorPoint2_px(-66, -58);
    cv::Point_<double> paintsDiff_m(0.08652, -0.0505);
    cv::Point_<double> k = (colorPoint1_px - colorPoint2_px);
    k.x = paintsDiff_m.x/k.x;
    k.y = paintsDiff_m.y/k.y;

    // Config for topic
    info.workMode = 4;
    info.kx = k.x;
    info.ky = k.y;
    info.topicName = "/Camera";
    info.cameraLinkName = "camera_link";
    info.brushTransform = -0.172;

    PainterCV cv(nh, info, freq);
    cv.loadServices();
    // int var = 0;
    // cv::Mat image;
    // ros::Rate r(freq);
    // while(ros::ok()) {
    //     cv.getImage(image);
    //     if (!image.empty())
    //         cv::imshow("Image", image);
    //     cv::waitKey(1);
    // }
    spinner.start();
    ros::waitForShutdown();

    // ros::spin();

    return 0;
}