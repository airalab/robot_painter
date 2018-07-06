#include <ros/ros.h>

#include <arm_manipulation/Manipulator.h>
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


#define DEBUG true
// TODO try using TF as main linear math.

size_t printedMarkers = 0;

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

void publishCanvasLink(geometry_msgs::TransformStamped transformStamped, double rate)
{
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    tf2_ros::TransformBroadcaster tf2_broadcaster;
    ros::Rate r(rate);

    ROS_INFO_STREAM("[LTP] Pubhlish /canvas_link with rate -- 1");
    while (ros::ok()) {
        transformStamped.header.stamp = ros::Time::now();

        tf2_broadcaster.sendTransform(transformStamped);
        // ROS_INFO_STREAM("Send Transform");
        // ROS_INFO_STREAM("t: \n" << transformStamped);
        r.sleep();
    }
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

Vector3d zRotation(Vector3d & v, double angle) {
    Vector3d result;
    result(0) = cos(angle)*v(0) + sin(angle)*v(1);
    result(1) = -sin(angle)*v(0) + cos(angle)*v(2);
    result(2) = v(2);

    return result;
}

Vector3d zRotation(std::vector<double> & v, double angle) {
    Vector3d result;
    result(0) = cos(angle)*v[0] + sin(angle)*v[1];
    result(1) = -sin(angle)*v[0] + cos(angle)*v[2];
    result(2) = v[2];

    return result;
}

// Функция рисования мазками
void drawSmear(kuka_cv::Color & colors, std::vector<kuka_cv::Color> & palette) {
    Pose p;
    p.position(0) = 0.1;
    p.position(1) = 0.4;
    p.position(2) = 0.85;
    p.orientation(0) = 0.0;
    p.orientation(1) = 0.0;
    p.orientation(2) = 1.5708;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "camera_test");
    ros::NodeHandle nh;

    // Service client
    ros::ServiceClient paletteClient = nh.serviceClient<kuka_cv::RequestPalette>("/request_palette");
    ros::ServiceClient canvasClient = nh.serviceClient<kuka_cv::RequestCanvas>("/request_canvas");
    ros::ServiceClient startImgProcClient = nh.serviceClient<std_srvs::Empty>("/start_image_preprocessing");
    ros::ServiceClient imgPaletteClient = nh.serviceClient<kuka_cv::RequestPalette>("/request_image_palette");
    ros::Publisher markerPublisher = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

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

    // Palette Colors
    kuka_cv::RequestPalette::Response palette;

    // Drawing image Colors
    std::vector<kuka_cv::Color> pictureColors;
    std::vector<kuka_cv::Pose>  pictureColorsPoses;

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

    kuka_cv::RequestPalette paletteInfo;
    kuka_cv::RequestCanvas canvasInfo;

    JointValues initiatlJV;
    initiatlJV(1) = -M_PI/2;
    initiatlJV(2) = M_PI/2;

    // Brush parameters
    Vector3d brushVector;
    brushVector(0) = 0.025; brushVector(1) = 0; brushVector(2) = -0.18;

    // Starting tf2 listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);


    int part = 0;
    std::cout << "Select part of test." << "\n"
        << "1  Palette part" << "\n"
        << "2  Canvas part" << "\n"
        << "3  Canvas image-proc part" << "\n"
        << "4  Draw" << "\n"
        << "Part (1, 2, 3, 4): ";
    std::cin >> part;

    // Params to determinate
    // -- camera vector
    // -- brush vector

    switch(part) {
        case 1: {

            if (!ros::service::waitForService("request_palette", ros::Duration(3.0))) {
                ROS_ERROR("Server request_palette is not active!");
                ros::shutdown();
            }
            // ROS_INFO_STREAM("[LTP] Move to initial pose");
            // manipulator.moveArm(initiatlJV);

            /// Palette detection and checking
            ROS_INFO_STREAM("[LTP] Move to Palette");
            manipulator.moveArm(detectPalettePose, config);
            ros::Duration(0.5).sleep();

            ROS_INFO_STREAM("[LTP] Receive palette message.");
            // 0 - measure mode; 1 - get data
            paletteInfo.request.mode = 0;
            do {
                if (paletteClient.call(paletteInfo)) {
                    ROS_INFO_STREAM("\t Successful");
                }
                palette = paletteInfo.response;
                ROS_WARN_STREAM("[LTP] Receive Colors array size = 0");
            } while (palette.colors.size() == 0 && ros::ok());

            ROS_INFO_STREAM("[LTP] Move to Colors on palette");
            Pose p;
            for (size_t i = 0; i < palette.colors.size(); ++i) {
                ROS_INFO_STREAM("[LTP] Number: " << (i + 1) << " --------------");
                ROS_INFO_STREAM("[LTP] Color: [" << (uint)palette.colors[i].r
                                        << ", " << (uint)palette.colors[i].g
                                        << ", " << (uint)palette.colors[i].b << "]");
                ROS_INFO_STREAM("[LTP] Pose: [" << palette.poses[i].x
                                        << ", " << palette.poses[i].y
                                        << ", " << palette.poses[i].z << "]");


                p.orientation(2) = 1.5708;
                p.position(0) = palette.poses[i].x;
                p.position(1) = palette.poses[i].y;
                p.position(2) = palette.poses[i].z;
                p.orientation(0) = palette.poses[i].phi;
                p.orientation(1) = palette.poses[i].theta;
                p.orientation(2) = palette.poses[i].psi;

                manipulator.moveArm(p, config);
            }
            break;
        }

        case 2: {

            /// Canvas detection
            ROS_INFO_STREAM("[LTP] Move to Canvas");
            manipulator.moveArm(detectCanvasPose, config);
            // ros::Duration(0.5).sleep();   // Movement duration. Drivers work is not correct!

            // cameraSrv.request.mode = 2;
            // ROS_INFO_STREAM("[LTP] Detect canvas transformation and dimensions: ");
            // if (cameraModeClient.call(cameraSrv)) {
            //     ROS_INFO_STREAM("\t Successful");
            // }

            canvasInfo.request.mode = 0;
            ROS_INFO_STREAM("[LTP] Receive canvas message: ");
            do {
                if (canvasClient.call(canvasInfo)) {
                    ROS_INFO_STREAM("\t Successful");
                    break;
                }
                ROS_WARN_STREAM("[LTP] Receive wrong canvas info");
            } while (canvasInfo.response.width == 0 && ros::ok());

            break;
        }

        case 3: {
            std_srvs::Empty emptyMsg;
            ROS_INFO_STREAM("[LTP] START image processing.");
            if (startImgProcClient.call(emptyMsg)) {
                ROS_INFO_STREAM("\t Successful");
            } else {
                ROS_ERROR_STREAM("\t ERROR");
                return 0;
            }

            ROS_INFO_STREAM("[LTP] Request information about pixels Color and position");
            if (imgPaletteClient.call(paletteInfo)) {
                ROS_INFO_STREAM("\t Successful");
            } else {
                ROS_ERROR_STREAM("\t ERROR");
            }
            pictureColors = paletteInfo.response.colors;
            pictureColorsPoses = paletteInfo.response.poses;

            visualization_msgs::Marker marker = createMarkerMsg(pictureColors, pictureColorsPoses);
            if (marker.points.empty()) {
                ROS_FATAL_STREAM("Picture pre processing Error: Markers is empty!");
                return 1;
            }

            canvasInfo.request.mode = 1;
            ROS_INFO_STREAM("[LTP] Receive canvas message: ");
            do {
                if (canvasClient.call(canvasInfo)) {
                    ROS_INFO_STREAM("\t Successful");
                    break;
                }
                ROS_WARN_STREAM("[LTP] Receive wrong canvas info");
            } while (canvasInfo.response.width == 0 && ros::ok());

            // Request canvas
            geometry_msgs::TransformStamped transform;
            transform.header.frame_id = "base_link";
            transform.child_frame_id  = "canvas_link";
            transform.transform.translation.x = canvasInfo.response.p.x;
            transform.transform.translation.y = canvasInfo.response.p.y;
            transform.transform.translation.z = canvasInfo.response.p.z;

            tf2::Quaternion q;
            q.setRPY(canvasInfo.response.p.phi, canvasInfo.response.p.psi, canvasInfo.response.p.theta);
            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            transform.transform.rotation.w = q.w();

            // Create thread that publish canvas link while ros is ok
            double rate = 1;
            boost::thread thr(publishCanvasLink, transform, rate);

            ros::Rate r(1);
            while (ros::ok()) {

                while (markerPublisher.getNumSubscribers() < 1)
                {
                  if (!ros::ok())
                  {
                    return 0;
                  }
                  ROS_WARN_ONCE("Please create a subscriber to the marker");
                  sleep(1);
                }
                markerPublisher.publish(marker);
                r.sleep();
            }
            thr.join();
            break;
        }

        case 4: {
            ROS_INFO_STREAM("[LTP] Move to initial pose");
            manipulator.moveArm(initiatlJV);

            paletteInfo.request.mode = 1;
            ROS_INFO_STREAM("[LTP] Receive palette message.");
            do {
                if (paletteClient.call(paletteInfo)) {
                    ROS_INFO_STREAM("\t Successful");
                }
                palette = paletteInfo.response;
                ROS_WARN_STREAM("[LTP] Receive Colors array size = 0");
            } while ((palette.colors.empty() || palette.poses.empty()) && ros::ok());

            canvasInfo.request.mode = 1;
            ROS_INFO_STREAM("[LTP] Receive canvas message: ");
            do {
                if (canvasClient.call(canvasInfo)) {
                    ROS_INFO_STREAM("\t Successful");
                }
                ROS_WARN_STREAM("[LTP] Receive wrong canvas info");
            } while (canvasInfo.response.width == 0 && ros::ok());
                        // Request canvas
            geometry_msgs::TransformStamped transform;
            transform.header.frame_id = "base_link";
            transform.child_frame_id  = "canvas_link";
            transform.transform.translation.x = canvasInfo.response.p.x;
            transform.transform.translation.y = canvasInfo.response.p.y;
            transform.transform.translation.z = canvasInfo.response.p.z;

            tf2::Quaternion q;
            q.setRPY(canvasInfo.response.p.phi, canvasInfo.response.p.psi, canvasInfo.response.p.theta);
            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            transform.transform.rotation.w = q.w();

            // Create thread that publish canvas link while ros is ok
            ROS_INFO_STREAM("Wait for transform /canvas_link");
            double rate = 1;
            boost::thread thrCnvs(publishCanvasLink, transform, rate);
            ros::Duration(5).sleep();

            ROS_INFO_STREAM("[LTP] Request information about pixels Color and position");
            if (imgPaletteClient.call(paletteInfo)) {
                ROS_INFO_STREAM("\t Successful");
            } else {
                ROS_ERROR_STREAM("\t ERROR");
                // return 0;
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

            rate = 3;
            ros::Rate rt(0.5);
            boost::thread thr(publishMarkers, marker, rate);


            geometry_msgs::TransformStamped transformStamped;
            try{
                for (size_t i = 0; i < 20; ++i) {
                    transformStamped = tfBuffer.lookupTransform("base_link", "canvas_link",
                        ros::Time(0), ros::Duration(3));
                }
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
            } double yaw = 0, pitch = 0, roll = 0;
            tf2::Quaternion canvasQ; tf2::Vector3 canvasV, picturePointV, pictureLocalV;
            tf2::fromMsg(transformStamped.transform.rotation, canvasQ);
            tf2::fromMsg(transformStamped.transform.translation, canvasV);
            tf2::Matrix3x3 rotMatrix(canvasQ);

            size_t count = 0;
            size_t currColorIndex = 0, prevColorIndex = 0;
            Pose poseForDrawing;
            Pose palettePose;

            // Color properties
            // We must use one "ideal" brush for measuring alive time. Brush coefficient is equal 1
            double aliveTime = 10;  // Number of swears for reduce color of paint to zero
            double brushCoeff = 1;  // Brush coeff. that contain reduce color of paint to zero if we using not ideal brush

            double paintingHeight = 0.05;   // Distance between brush and work ground

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
                    ROS_INFO("Select color! (%d | %d)", paletteSize, currColorIndex);
                }
                ROS_WARN("Select color! (%d | %d)", paletteSize, currColorIndex);

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
                // *** Local Control Circle
                // ** Get the paint
                // Move under the paints
                palettePose.position(0) = palette.poses[currColorIndex].x;
                palettePose.position(1) = palette.poses[currColorIndex].y;
                palettePose.position(2) = palette.poses[currColorIndex].z;
                palettePose.orientation(0) = palette.poses[currColorIndex].phi;
                palettePose.orientation(1) = palette.poses[currColorIndex].theta;
                palettePose.orientation(2) = palette.poses[currColorIndex].psi;
                // palettePose.position -= zRotation(brushVector, -1.5708);
                // palettePose.orientation(2) = 1.5708;
                manipulator.moveArm(palettePose, config);

                // Move to desired paint
                palettePose.position(2) -= paintingHeight;
                manipulator.moveArm(palettePose, config);
                ros::Duration(0.5).sleep();

                // ** Draw the paint
                // Move under canvas
                pictureLocalV = tf2::Vector3(pictureColorsPoses[printedMarkers].x, pictureColorsPoses[printedMarkers].y, pictureColorsPoses[printedMarkers].z);
                picturePointV = rotMatrix*pictureLocalV + canvasV;

                poseForDrawing.position(0) = picturePointV.m_floats[0];
                poseForDrawing.position(1) = picturePointV.m_floats[1];
                poseForDrawing.position(2) = picturePointV.m_floats[2] + paintingHeight;
                // poseForDrawing.position -= zRotation(brushVector, 1.5708);
                poseForDrawing.orientation(2) = -1.5708;
                if (DEBUG) {
                    ROS_INFO_STREAM("[POINT] transform: (" << poseForDrawing.position(0)
                        << ", " << poseForDrawing.position(1)
                        << ", " << poseForDrawing.position(2) << ")");
                }
                manipulator.moveArm(poseForDrawing, config);


                // Paint the color
                poseForDrawing.position(2) -= paintingHeight;
                manipulator.moveArm(poseForDrawing, config);
                ++printedMarkers;
                thr.interrupt();
                ros::Duration(1).sleep();

                rt.sleep();
            }
            thr.join();
        }
    }
    return 0;
}