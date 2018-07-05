#include <CameraTransforms.h>
#define CALIBRATION false

CameraTransforms::CameraTransforms(ros::NodeHandle & nh, CameraWorkingInfo info)
: tfBuffer(ros::Duration(10)), camLinkName(info.cameraLinkName)
{
    tfListener = new tf2_ros::TransformListener(tfBuffer, nh);

    kx = info.kx; ky = info.ky;
    brushTransl.m_floats[2] = info.brushTransform;
}
CameraTransforms::~CameraTransforms()
{}

void CameraTransforms::updateFrame()
{
    // Function tetermine rotation matrix and translation vector
    // For transform point function

    geometry_msgs::TransformStamped cameraTransform, holderTransform;
    tf2::Quaternion camQuat, holderQuat;
    tf2::Vector3 holderTransl, camTransl;
    tf2::Matrix3x3 holderRot;

    // Take transforms
    cameraTransform = tfBuffer.lookupTransform("base_link", camLinkName, ros::Time(0), ros::Duration(3));
    holderTransform = tfBuffer.lookupTransform("base_link", "holder_link", ros::Time(0), ros::Duration(3));
    // link6Transform  = tfBuffer.lookupTransform("base_link", "link_6", ros::Time(0), ros::Duration(3));

    // std::cout << "Info: " << cameraTransform << std::endl;
    // std::cout << "Info: " << holderTransform << std::endl;
    // std::cout << "Info: " << link6Transform << std::endl;

    // Transform ROS message to tf2 data type
    tf2::fromMsg(cameraTransform.transform.rotation, camQuat);
    tf2::fromMsg(cameraTransform.transform.translation, camTransl);
    tf2::fromMsg(holderTransform.transform.rotation, holderQuat);
    tf2::fromMsg(holderTransform.transform.translation, holderTransl);
    // tf2::fromMsg(link6Transform.transform.translation, translation);
    holderRot = tf2::Matrix3x3(holderQuat);
    rotation = tf2::Matrix3x3(camQuat);

    translation = holderTransl - holderRot*brushTransl;
    // std::cout << "translation (" << translation.m_floats[0] << ", " << translation.m_floats[1] << ", " << translation.m_floats[2] << ")" << std::endl;
}

void CameraTransforms::transformPoint(kuka_cv::Pose & p)
{
    // Rotate default camera frame
    kuka_cv::Pose p2;
    if (CALIBRATION) {
        kx = 1; ky = 1;
    }
    p2.x = -ky*(p.y - camHeight/2);
    p2.y = kx*(p.x - camWidth/2);
    p2.z = -p.z;
    tf2::Vector3 temp(p2.x, p2.y, p2.z), test;

    // Transform point from camera to base link
    if (!CALIBRATION)
        temp = rotation*temp + translation;
    // test = rotation*temp;

    p.x = temp.m_floats[0];
    p.y = temp.m_floats[1];
    p.z = temp.m_floats[2];
    getRPY(p.phi, p.theta, p.psi);
}

void CameraTransforms::setCameraDimensions(double w, double h)
{
    camWidth = w; camHeight = h;
}

void CameraTransforms::getRPY(double & roll, double & pitch, double & yaw)
{
    // Transform tf2::Matrix3x3 to rpy angles
    rotation.getRPY(roll, pitch, yaw);
}