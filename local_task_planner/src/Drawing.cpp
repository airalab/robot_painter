#include <ros/ros.h>

#include <kuka_manipulation_moveit/KukaMoveit.hpp>
#include <kuka_cv/Color.h>
#include <kuka_cv/RequestPalette.h>
#include <kuka_cv/RequestCanvas.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <boost/thread/thread.hpp>

// TF
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/impl/utils.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_msgs/String.h>


#define DEBUG false
// TODO try using TF as main linear math.

bool start = false;
size_t printedMarkers = 0;
tf2::Matrix3x3 R;
tf2::Vector3 v;
const double COLOR_BOTLE_HEIGHT = 0.06;
const double COLOR_HEIGHT = 0.045;
const double HEIGHT_OFFSET = COLOR_BOTLE_HEIGHT - COLOR_HEIGHT + 0.02;
const double BRUSH_HEIGHT = 0.01;
const double BRUSH_WIDTH = 0.01;

visualization_msgs::Marker createMarkerMsg(std::vector<kuka_cv::Color> & colors, std::vector<kuka_cv::Pose> & poses) {

    if (colors.empty() || poses.empty()) {
        ROS_FATAL_STREAM("Picture pre processing Error: Empty respospone!");
        ros::shutdown();
        return visualization_msgs::Marker();
    }

    uint32_t shape = visualization_msgs::Marker::SPHERE_LIST;
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/canvas_link";
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
    for (uint32_t i = 0; i < colors.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = poses[i].x;
        p.y = poses[i].y;
        p.z = poses[i].z;

        std_msgs::ColorRGBA color;
        color.r = 1.0*colors[i].r/255;
        color.g = 1.0*colors[i].g/255;
        color.b = 1.0*colors[i].b/255;
        color.a = 1.0;

        marker.points.push_back(p);
        marker.colors.push_back(color);
    }

    return marker;
}

void publishMarkers(visualization_msgs::Marker & marker, size_t rate) {
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    ros::Publisher pub = node->advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    visualization_msgs::Marker activeMarkers;
    activeMarkers.header.frame_id = marker.header.frame_id;
    activeMarkers.header.stamp = ros::Time::now();
    activeMarkers.ns = marker.ns;
    activeMarkers.id = marker.id;
    activeMarkers.type = marker.type;
    activeMarkers.action = marker.action;
    activeMarkers.scale.x = marker.scale.x;
    activeMarkers.scale.y = marker.scale.y;
    activeMarkers.scale.z = marker.scale.z;
    activeMarkers.lifetime = marker.lifetime;

    size_t prevValue = printedMarkers;
    // ROS_INFO_STREAM("printedMarkers:" << printedMarkers);

    if (printedMarkers != 0) {
        for (size_t i = 0; i < printedMarkers; ++i) {
            activeMarkers.points.push_back(marker.points[i]);
            activeMarkers.colors.push_back(marker.colors[i]);
        }
    }

    ROS_INFO_STREAM("[LTP] Start marker publishing");
    ros::Rate r(rate);
    while (ros::ok()) {
        while (pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        activeMarkers.header.stamp = ros::Time::now();
        if (printedMarkers - prevValue == 1) {
            activeMarkers.points.push_back(marker.points[printedMarkers - 1]);
            activeMarkers.colors.push_back(marker.colors[printedMarkers - 1]);
            prevValue = printedMarkers;
        } else if (printedMarkers - prevValue > 1) {
            ROS_ERROR_STREAM("Markers ERROR.");
        }
        pub.publish(activeMarkers);
        r.sleep();
    }
}

void collectPaintOnBrush(KukaMoveit & manipulator, kuka_cv::Pose & pose)
{
    geometry_msgs::Pose p;

    p.position.x = pose.x;
    p.position.y = pose.y;
    p.position.z = pose.z + COLOR_BOTLE_HEIGHT + HEIGHT_OFFSET;
    p.orientation.w = 1;
    manipulator.move(p, DEBUG);

    p.position.z = pose.z + COLOR_HEIGHT - BRUSH_HEIGHT;
    manipulator.move(p, DEBUG);

    p.position.z = pose.z + COLOR_BOTLE_HEIGHT + HEIGHT_OFFSET;
    manipulator.move(p, DEBUG);
}


void doSmear(KukaMoveit & manipulator, kuka_cv::Pose & pose)
{
    geometry_msgs::Pose p;

    p.position.x = pose.x;
    p.position.y = pose.y;
    p.position.z = pose.z + HEIGHT_OFFSET;
    p.orientation.w = 1;

    ROS_INFO_STREAM("P: [" << p.position.x << ", " << p.position.y << ", " << p.position.z << "]");

    manipulator.move(p, DEBUG);

    p.position.z = pose.z;
    manipulator.move(p, DEBUG);

    p.position.x -= BRUSH_WIDTH;
    manipulator.move(p, DEBUG);

    p.position.z = pose.z + COLOR_BOTLE_HEIGHT + HEIGHT_OFFSET;
    manipulator.move(p, DEBUG);

}

void transformPointFromCanvasToBase(kuka_cv::Pose & pose) {

    // Transform:
    // R*p + v
    // R - canvas transform matrix, v - translation or canvas frame, p - painting point

    tf2::Vector3 result, p(pose.x, pose.y, pose.z);
    result = R*p + v;

    pose.x = result.m_floats[0];
    pose.y = result.m_floats[1];
    pose.z = result.m_floats[2];
    pose.phi = 0; pose.theta = 0; pose.psi = 0;
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    start = true;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "camera_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();


    // Service client
    ros::ServiceClient paletteClient = nh.serviceClient<kuka_cv::RequestPalette>("/request_palette");
    ros::ServiceClient canvasClient = nh.serviceClient<kuka_cv::RequestCanvas>("/request_canvas");
    ros::ServiceClient startImgProcClient = nh.serviceClient<std_srvs::Empty>("/start_image_preprocessing");
    ros::ServiceClient imgPaletteClient = nh.serviceClient<kuka_cv::RequestPalette>("/request_image_palette");
    ros::Publisher markerPublisher = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

    /* AIRA Stack */
    ros::Subscriber runSubscriber = nh.subscribe("run", 10, chatterCallback);
    ros::ServiceClient liabilityFinishClient = nh.serviceClient<std_srvs::Empty>("liability/finish");

    // Initialize manipulator
    KukaMoveit manipulator("manipulator");

    /* Set joint constraints */

    // Joint a1
    moveit_msgs::Constraints constraints;
    constraints.joint_constraints.resize(2);
    constraints.joint_constraints[0].joint_name = "joint_a1";
    constraints.joint_constraints[0].position = 0.0;
    constraints.joint_constraints[0].tolerance_above = 1;
    constraints.joint_constraints[0].tolerance_below = 1;
    constraints.joint_constraints[0].weight = 1.0;

    // Joint a4
    constraints.joint_constraints[1].joint_name = "joint_a4";
    constraints.joint_constraints[1].position = 0.0;
    constraints.joint_constraints[1].tolerance_above = 1;
    constraints.joint_constraints[1].tolerance_below = 1;
    constraints.joint_constraints[1].weight = 1.0;

    manipulator.getMoveGroup()->setPathConstraints(constraints);

    kuka_cv::RequestPalette::Response palette;
    kuka_cv::RequestPalette paletteInfo;
    kuka_cv::RequestCanvas canvasInfo;

    // Drawing image Colors
    std::vector<kuka_cv::Color> pictureColors;
    std::vector<kuka_cv::Pose>  pictureColorsPoses;

    while(ros::ok()) {
        if (start) {
             /* Palette info */
            paletteInfo.request.mode = 0;
            ROS_INFO_STREAM("[LTP] Receive palette message.");
            do {
                if (paletteClient.call(paletteInfo)) {
                    palette = paletteInfo.response;
                    break;
                }
                ROS_WARN_STREAM("[LTP] Receive Colors array size = 0");
            } while ((palette.colors.empty() || palette.poses.empty()) && ros::ok());

            /* Canvas info */
            canvasInfo.request.mode = 0;
            ROS_INFO_STREAM("[LTP] Receive canvas message: ");
            do {
                if (canvasClient.call(canvasInfo)) {
                    break;
                }
                ROS_WARN_STREAM("[LTP] Receive wrong canvas info");
            } while (canvasInfo.response.width == 0 && ros::ok());
            R.setRPY(canvasInfo.response.p.phi, canvasInfo.response.p.theta, canvasInfo.response.p.psi);
            v = tf2::Vector3(canvasInfo.response.p.x, canvasInfo.response.p.y, canvasInfo.response.p.z);

            /* Image palette info */
            std_srvs::Empty emptyMsg;
            ROS_INFO_STREAM("[LTP] START image processing.");
            if (!startImgProcClient.call(emptyMsg)) {
                ROS_ERROR_STREAM("\t ERROR");
                return 0;
            }
            ROS_INFO_STREAM("[LTP] Request information about pixels Color and position");
            if (!imgPaletteClient.call(paletteInfo)) {
                ROS_ERROR_STREAM("\t ERROR");
                return 0;
            }
            pictureColors = paletteInfo.response.colors;
            pictureColorsPoses = paletteInfo.response.poses;

            ROS_INFO_STREAM("[LTP] Start Drawing...");
            // Draw Params
            bool isDraw = true;
            bool updatePaint = true;


            visualization_msgs::Marker marker = createMarkerMsg(pictureColors, pictureColorsPoses);

            size_t pxNum = pictureColors.size();
            size_t paletteSize = palette.colors.size();
            ROS_INFO_STREAM("[LTP] Points number: " << pxNum);

            size_t rate = 3;
            ros::Rate rt(0.5);
            boost::thread thr(publishMarkers, marker, rate);

            size_t swearsNumber = 0;
            size_t currColorIndex = 0, prevColorIndex = 0;

            // Color properties
            // We must use one "ideal" brush for measuring alive time. Brush coefficient is equal 1
            double aliveTime = 4;  // Number of swears for reduce color of paint to zero
            double brushCoeff = 1;  // Brush coeff. that contain reduce color of paint to zero if we using not ideal brush

            // Global drawing circle
            while (ros::ok() && isDraw) {

                if (printedMarkers == pxNum) {
                    ROS_ERROR("printedMarkers == pxNum");
                    isDraw = false;
                }

                // Find color in palette
                prevColorIndex = currColorIndex;
                while (currColorIndex < paletteSize &&
                    (pictureColors[printedMarkers].r != palette.colors[currColorIndex].r ||
                     pictureColors[printedMarkers].g != palette.colors[currColorIndex].g ||
                     pictureColors[printedMarkers].b != palette.colors[currColorIndex].b))
                {
                    ++currColorIndex;
                    ROS_INFO("Select color! (%lu | %lu)", paletteSize, currColorIndex);
                }
                ROS_WARN("Select color! (%lu | %lu)", paletteSize, currColorIndex);

                if (currColorIndex == paletteSize) {
                    currColorIndex = 0;
                    continue;
                } else if (currColorIndex > paletteSize) {
                    ROS_ERROR_STREAM("Error of changing palette color.");
                }

                if (DEBUG) {
                    ROS_INFO_STREAM("Count: " << printedMarkers);
                    ROS_INFO_STREAM("[COLOR] palette: ["
                        << (uint)palette.colors[currColorIndex].b << ","
                        << (uint)palette.colors[currColorIndex].g << ","
                        << (uint)palette.colors[currColorIndex].r << "] vs ("
                        << (uint)pictureColors[printedMarkers].b << ","
                        << (uint)pictureColors[printedMarkers].g << ","
                        << (uint)pictureColors[printedMarkers].r << ")");
                }

                if (swearsNumber >= aliveTime || swearsNumber == 0) {
                    collectPaintOnBrush(manipulator, palette.poses[currColorIndex]);
                    swearsNumber = 0;
                }

                transformPointFromCanvasToBase(pictureColorsPoses[printedMarkers]);
                doSmear(manipulator, pictureColorsPoses[printedMarkers]);

                ROS_INFO_STREAM("[LTP] POINT (" << printedMarkers << ")");
                ++printedMarkers;
                ++swearsNumber;
                thr.interrupt();

            }
            thr.join();

            std_srvs::Empty empty;
            if (!liabilityFinishClient.call(empty)) {
                return 0;
            }
            start = false;
        }
    }
    return 0;
}