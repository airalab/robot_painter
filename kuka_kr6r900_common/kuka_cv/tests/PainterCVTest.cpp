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
    info.workMode = 4;
    // info.deviceNumber = 0;
    // info.imagePath = "/home/senex/Tkests/RealsenseCV/SavedImages/Image.jpg";
    info.topicName = "/Camera";

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