#include <kuka_cv/PainterCV.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "cv_test");
    ros::NodeHandle nh;

    // ros::AsyncSpinner spinner(4);
    int freq = 10; // in hz (image updating frequency)

    // Configure camera work
    // Work modes::
    // 1 |  from Image      (set imagePath)
    // 2 |  from Web Camera (set deviceNumber)
    // 3 |  from Realsense  (nothing)
    // 4 |  from Topic      (set topicName)

    CameraWorkingInfo info;
    info.workMode = 4;
    info.topicName = "/Camera";

    PainterCV cv(nh, info, freq);
    cv::Mat image;
    std::vector<cv::Point> p;
    std::vector<cv::Vec3b> c;
    std::vector<float> radii;

    while(ros::ok()) {
        cv.getImage(image);
        if (!image.empty()) {
            findCirclesByCanny(image, radii, p);
            std::cout << "[size] points: " << p.size() << std::endl;
            if (p.size() == 1) {
                for (size_t pt = 0; pt < p.size(); ++pt)
                    cv::circle(image, p[pt], radii[pt], cv::Scalar(255), 2);
            }
            cv::imshow("Image", image);
        }

        cv::waitKey(1);
        ros::spinOnce();
    }

    return 0;
}