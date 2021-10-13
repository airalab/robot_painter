#include <ros/ros.h>
#include <ros/console.h>
#include <kuka_manipulation_moveit/KukaMoveit.hpp>

#include <termios.h>

// TF
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/impl/utils.h>
#include <tf2/utils.h>

// Messages
#include <kuka_cv/RequestCanvas.h>
#include <geometry_msgs/Pose.h>

#define POINTS_NUM 30
#define CANVAS_FRAME_NAME "base_link"
ros::Publisher markerPublisher;

// Major
void findVertices(const kuka_cv::RequestCanvas::Response info, std::vector<tf2::Vector3> & points);
void calculatePath(const std::vector<tf2::Vector3> & p, std::vector<geometry_msgs::Pose> & wp);
void visualizate(const std::vector<geometry_msgs::Pose> & poses);

// Utils
void toMsg(geometry_msgs::Pose & pose, const tf2::Vector3 & point);
int getch();

int main(int argc, char ** argv) {

    ros::init(argc, argv, "canvas_drawer");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Set logger level to debug
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);


    std::vector<geometry_msgs::Pose> waypoints;
    std::vector<tf2::Vector3> points(4);
    std::string serviceName;
    ros::Rate r(1);

    // Define and waiting the service
    serviceName = "/request_canvas";
    ros::ServiceClient canvasClient = nh.serviceClient<kuka_cv::RequestCanvas>(serviceName);
    while (ros::ok() && !ros::service::waitForService(serviceName)) {
        ROS_WARN_ONCE("Service %s is not ready!", serviceName.c_str());
        r.sleep();
    }

    markerPublisher = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    // Declare messages
    kuka_cv::RequestCanvas canvasInfo;
    if (!canvasClient.call(canvasInfo)) {
        return 1;
    }

    ROS_DEBUG_STREAM("Information of CANVAS \n" << canvasInfo.response);

    findVertices(canvasInfo.response, points);
    calculatePath(points, waypoints);
    visualizate(waypoints);

    // TODO realize the trajectory
    KukaMoveit manipulator("manipulator");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    manipulator.getMoveGroup()->setMaxVelocityScalingFactor(0.5);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    double fraction = manipulator.getMoveGroup()->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory
    );

    plan.trajectory_ = trajectory;
    manipulator.getMoveGroup()->execute(plan);

    visualizate(waypoints);

    return 0;
}

void findVertices(const kuka_cv::RequestCanvas::Response info, std::vector<tf2::Vector3> & points)
{
    // Variables
    tf2::Transform H;
    tf2::Quaternion q;
    q.setRPY(info.p.phi, info.p.theta, info.p.psi);

    H.setOrigin(tf2::Vector3(info.p.x, info.p.y, info.p.z));
    H.setRotation(q);

    // Starts with top right point
    points[0] = H*tf2::Vector3(-info.height/2,  info.width/2, 0);
    points[1] = H*tf2::Vector3( info.height/2,  info.width/2, 0);
    points[2] = H*tf2::Vector3( info.height/2, -info.width/2, 0);
    points[3] = H*tf2::Vector3(-info.height/2, -info.width/2, 0);

    ROS_DEBUG_STREAM("Information of POINTS" << "\n"
        "1: [" << points[0].m_floats[0] << ", " << points[0].m_floats[1] << ", " << points[0].m_floats[2] << "]\n"
        "2: [" << points[1].m_floats[0] << ", " << points[1].m_floats[1] << ", " << points[1].m_floats[2] << "]\n"
        "3: [" << points[2].m_floats[0] << ", " << points[2].m_floats[1] << ", " << points[2].m_floats[2] << "]\n"
        "4: [" << points[3].m_floats[0] << ", " << points[3].m_floats[1] << ", " << points[3].m_floats[2] << "]\n");
}

void calculatePath(const std::vector<tf2::Vector3> & p, std::vector<geometry_msgs::Pose> & wp)
{
    tf2::Vector3 diff;
    geometry_msgs::Pose pose;
    uint size = p.size();

    for (size_t i = 1; i < size; ++i) {
        diff = (p[i] - p[i-1])/(POINTS_NUM - 1);
        for (size_t j = 0; j < POINTS_NUM - 1; ++j) {
            toMsg(pose, diff*j + p[i-1]);
            wp.push_back(pose);
        }
    }

    diff = (p[0] - p[size-1])/(POINTS_NUM - 1);
    for (size_t j = 0; j < POINTS_NUM; ++j) {
        toMsg(pose, diff*j + p[size-1]);
        wp.push_back(pose);
    }

    ROS_DEBUG_STREAM("The last point of WAYPOINTS [" << wp.size() << "]\n" << wp[wp.size()-1]);
}

void visualizate(const std::vector<geometry_msgs::Pose> & poses) {

    if (poses.empty()) {
        ROS_ERROR("Array of WAYPOINTS if empty!");
        return;
    }

    uint32_t shape = visualization_msgs::Marker::SPHERE_LIST;
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = CANVAS_FRAME_NAME;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.lifetime = ros::Duration(1);

    // Create the vertices for the points and lines
    std_msgs::ColorRGBA color;
    color.a = 1;
    for (uint32_t i = 0; i < poses.size(); ++i)
    {
        marker.points.push_back(poses[i].position);
        marker.colors.push_back(color);
    }

    ros::Rate rate(1);

    ROS_INFO("Start marker publisher!");
    while (ros::ok() && getch() != 115) {
        while (markerPublisher.getNumSubscribers() < 1)
        {
            ROS_INFO_STREAM(markerPublisher.getNumSubscribers());
            if (!ros::ok())
            {
                return;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

        markerPublisher.publish(marker);
        rate.sleep();
    }
}



void toMsg(geometry_msgs::Pose & pose, const tf2::Vector3 & point)
{
    pose.position.x = point.m_floats[0];
    pose.position.y = point.m_floats[1];
    pose.position.z = point.m_floats[2];
}

int getch()
{
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);               // save old settings
    newt = oldt;
    newt.c_lflag &=~ (ICANON | ECHO);
    newt.c_cc[VEOL] = 1;
    newt.c_cc[VEOF] = 2;
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);      // apply new settings

    int c = std::getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);      // restore old settings
    return c;
}
