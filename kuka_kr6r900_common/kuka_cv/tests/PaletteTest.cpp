#include <kuka_cv/PainterCV.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#define TEST_Z 0.85

void showPalette(cv::Mat image, std::vector<cv::Point> & p, std::vector<kuka_cv::Pose> & pTransformed) {

    std::string windowName = "circles";
    int height = image.rows;
    int width = image.cols;

    ///// Default axis
    //
    //     ─┼───────────━━ x
    //      │
    //      │
    //      │
    //      │
    //      ┃
    //      y

    // x axis
    cv::arrowedLine(image, cv::Point(width/2, 0), cv::Point(width/2, height), cv::Scalar(255, 0, 0), 2, 8, 0, 0.01);
    cv::putText(image, cv::String("X"), cv::Point(width/2 + 20, height), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0, 0, 0), 2);

    // y axis
    cv::arrowedLine(image, cv::Point(0, height/2), cv::Point(width, height/2), cv::Scalar(255, 0, 0), 2, 8, 0, 0.01);
    cv::putText(image, cv::String("Y"), cv::Point(width, height/2 + 20), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0, 0, 0), 2);

    std::stringstream ss;
    for (size_t i = 0; i < p.size(); ++i) {
        ss << "(" << pTransformed[i].x << ", " << pTransformed[i].y << ")";
        cv::putText(image, cv::String(ss.str()), p[i], cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0, 0, 0), 2);
        ss.str("");
    }

    // Draw image
    cv::imshow(windowName, image);
    cv::waitKey(0);
}

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

    // Calculation
    cv::Point_<double> colorPoint1_px(122, 50);
    cv::Point_<double> colorPoint2_px(56, -64);
    cv::Point_<double> paintsDiff_m(0.0505, 0.08652);
    cv::Point_<double> k = (colorPoint1_px - colorPoint2_px);
    k.x = paintsDiff_m.x/k.x;
    k.y = paintsDiff_m.y/k.y;

    std::cout << "k: " << k << std::endl;

    // Config for topic
    CameraWorkingInfo info;
    info.workMode = 4;
    info.kx = k.x;
    info.ky = k.y;
    info.topicName = "/Camera";
    info.cameraLinkName = "camera_link";
    info.brushTransform = -0.172;

    PainterCV cv(nh, info, freq);

    // TF2 broakcaster for palette colors
    tf2_ros::TransformBroadcaster br;

    while(ros::ok()) {
        cv::Mat image = cv.getImage().clone();

        if (!image.empty()) {

            std::vector<cv::Point> p;   // Palette position of center
            std::vector<kuka_cv::Pose> p1;
            std::vector<cv::Vec3b> c;   // Paints colors
            kuka_cv::Color color;
            kuka_cv::Pose pose;
            geometry_msgs::TransformStamped transformStamped;
            tf2::Quaternion q;

            // Update camera frame
            cv.camTransforms.updateFrame();

            // Recognize circle colors at palette
            if (!cv.detectPaletteColors(image, p, c)) {
                ROS_ERROR("No circles was found!");
                continue;
            }

            p1.resize(p.size());
            for (size_t i = 0; i < p.size(); ++i) {
                // TODO check color
                color.r = c[i][0];
                color.g = c[i][1];
                color.b = c[i][2];
                pose.x =  p[i].x;
                pose.y =  p[i].y;
                pose.z =  TEST_Z;

                std::stringstream ss;

                ss << "color_" << i;

                cv.camTransforms.transformPoint(pose);

                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = "/base_link";
                transformStamped.child_frame_id = ss.str();
                transformStamped.transform.translation.x = pose.x;
                transformStamped.transform.translation.y = pose.y;
                transformStamped.transform.translation.z = pose.z;
                q.setRPY(0, 0, 0);
                transformStamped.transform.rotation.x = q.x();
                transformStamped.transform.rotation.y = q.y();
                transformStamped.transform.rotation.z = q.z();
                transformStamped.transform.rotation.w = q.w();
                std::cout << "Transform:\n" << transformStamped << std::endl;

                p1[i] = pose;

                br.sendTransform(transformStamped);
            }
            std::cout << "------------------------------------ iter ----------------------------------" << std::endl;
            showPalette(image, p, p1);
        }

        cv::waitKey(1);
        ros::spinOnce();
    }

    return 0;
}