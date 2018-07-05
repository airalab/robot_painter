#ifndef CAMERA_TRANSFORMS
#define CAMERA_TRANSFORMS

#include <CommonCV.h>

// ROS
#include <ros/ros.h>

// Messages
#include <geometry_msgs/TransformStamped.h>
#include <kuka_cv/Pose.h>

// TF
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/impl/utils.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>


class CameraTransforms
{
    public:
        CameraTransforms(ros::NodeHandle & nh, CameraWorkingInfo info);
        ~CameraTransforms();

        void updateFrame();
        void transformPoint(kuka_cv::Pose & p);
        void setCameraDimensions(double w, double h);
        void getRPY(double & roll, double & pitch, double & yaw);

    private:

        tf2::Vector3 translation;
        tf2::Matrix3x3 rotation;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener* tfListener;

        std::string camLinkName;

        /// Camera resolution
        double camWidth, camHeight;
        /// Camera m/px coefficients
        double kx, ky;

        // Brush translation vector
        tf2::Vector3 brushTransl;
};

#endif