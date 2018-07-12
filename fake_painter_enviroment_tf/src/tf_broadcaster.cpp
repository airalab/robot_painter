#include <ros/ros.h>

// TF
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>

#include <kuka_cv/Color.h>
#include <kuka_cv/Pose.h>
#include <kuka_cv/RequestCanvas.h>
#include <kuka_cv/RequestPalette.h>

/// Main information
kuka_cv::RequestPalette::Response palette;
kuka_cv::RequestCanvas::Response canvas;

// Plane constants
const double A = -0.0641;
const double B = 0.0214;
const double C = 0.9977;
const double D = -0.2198;

// Canvas transform
const double px = 0.52;
const double py = -0.24;
const double qx = -0.011;
const double qy = -0.032;
const double qz = 0.0;
const double qw = 0.999;

geometry_msgs::TransformStamped getCanvasTransform()
{
    geometry_msgs::TransformStamped canvasTransform;
    canvasTransform.header.frame_id = "base_link";
    canvasTransform.child_frame_id = "canvas_link";
    canvasTransform.transform.translation.x = px;
    canvasTransform.transform.translation.y = py;
    canvasTransform.transform.translation.z = -(A*px + B*px + D)/C;
    canvasTransform.transform.rotation.x = qx;
    canvasTransform.transform.rotation.y = qy;
    canvasTransform.transform.rotation.z = qz;
    canvasTransform.transform.rotation.w = qw;

    return canvasTransform;
}

std::vector<geometry_msgs::TransformStamped> getPaletteTransform()
{
    std::vector<geometry_msgs::TransformStamped> paletteTransforms;

    geometry_msgs::TransformStamped colorTransform;
    colorTransform.header.frame_id = "base_link";
    colorTransform.child_frame_id = "color_1";
    colorTransform.transform.translation.x = 0.5;
    colorTransform.transform.translation.y = 0.2;
    colorTransform.transform.translation.z = 0.258;
    colorTransform.transform.rotation.x = 0;
    colorTransform.transform.rotation.y = 0;
    colorTransform.transform.rotation.z = 0;
    colorTransform.transform.rotation.w = 1;

    paletteTransforms.push_back(colorTransform);

    return paletteTransforms;
}

bool canvasCallback(kuka_cv::RequestCanvas::Request  & req,
                               kuka_cv::RequestCanvas::Response & res)
{
    // Modes:
    // 0 - measure
    // 1 - get info

    if (req.mode == 0) {
        res = canvas;
    } else if (req.mode == 1) {
        res = canvas;
    }

    return true;
}
bool paletteCallback(kuka_cv::RequestPalette::Request  & req,
                     kuka_cv::RequestPalette::Response & res)
{
    // Modes:
    // 0 - measure
    // 1 - get info

    if (req.mode == 0) {
        res = palette;
    } else if (req.mode == 1) {
        res = palette;
    }
    return true;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "painter_broadcaster");
    ros::NodeHandle nh;

    size_t rate = 5;
    tf2_ros::StaticTransformBroadcaster tf2_broadcaster;
    ros::Rate r(rate);

    ros::ServiceServer canvasServer = nh.advertiseService("request_canvas", canvasCallback);
    ros::ServiceServer paletteService = nh.advertiseService("request_palette", paletteCallback);

    std::vector<geometry_msgs::TransformStamped> paletteTransforms = getPaletteTransform();
    geometry_msgs::TransformStamped paletteTransformStamped = paletteTransforms[0];
    geometry_msgs::TransformStamped canvasTransformStamped = getCanvasTransform();

    // Calculate euler angles of transform
    double roll, pitch, yaw;
    tf2::Quaternion q(qx, qy, qz, qw);
    tf2::Matrix3x3 R(q);
    R.getRPY(roll, pitch, yaw);

    // FIll kuka_cv messages
    /* Canvas */
    kuka_cv::Pose p;
    p.x = canvasTransformStamped.transform.translation.x;
    p.y = canvasTransformStamped.transform.translation.y;
    p.z = canvasTransformStamped.transform.translation.z;
    p.phi = roll; p.theta = pitch; p.psi = yaw;
    canvas.p = p;
    canvas.width = 0.210 - 2*0.04;
    canvas.height = 0.297 - 2*0.04;

    /* Palette */
    kuka_cv::Color color;
    kuka_cv::Pose pose;
    pose.x = paletteTransformStamped.transform.translation.x;
    pose.y = paletteTransformStamped.transform.translation.y;
    pose.z = paletteTransformStamped.transform.translation.z;
    pose.phi = 0; pose.theta = 0; pose.psi = 0;
    color.r = 10; color.g = 10; color.b = 10;
    palette.poses.push_back(pose);
    palette.colors.push_back(color);

    ROS_INFO_STREAM("[LTP] Pubhlish /canvas_link with rate -- " << rate);
    ROS_INFO_STREAM("[LTP] Pubhlish /color_1 with rate -- " << rate);
    while (ros::ok()) {
        paletteTransformStamped.header.stamp = ros::Time::now();

        tf2_broadcaster.sendTransform(paletteTransformStamped);
        tf2_broadcaster.sendTransform(canvasTransformStamped);

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
