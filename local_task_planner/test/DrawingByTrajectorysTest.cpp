#include <ros/ros.h>
#include <ros/package.h>

#include <kuka_manipulation_moveit/KukaMoveit.hpp>
#include <kuka_cv/Color.h>
#include <kuka_cv/RequestPalette.h>
#include <kuka_cv/RequestCanvas.h>
#include <local_task_planner/TextConverterService.h>
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
#include <geometry_msgs/PoseArray.h>

#include <std_msgs/String.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#define START_POSE_DEBUG false
#define DEBUG false
// TODO try using TF as main linear math.

bool start = false;
size_t printedMarkers = 0;
tf2::Matrix3x3 R;
tf2::Vector3 v;
std_msgs::String msg;
const double COLOR_BOTLE_HEIGHT = 0.08;
const double COLOR_HEIGHT = 0.026;
const double HEIGHT_OFFSET = 0.05;
//brushes sizes
//const double BRUSH_HEIGHT = 0.215; //from j6 center big blue brush
const double BRUSH_HEIGHT = 0.2; //from j6 center small white brush

const double BRUSH_WIDTH = 0.01;


const std::string BAG_FILE_PATH = ros::package::getPath("picture_preprocessing") + "/data/test.bag";
const std::string TRAJECTORY_TOPIC_NAME = "/path";

local_task_planner::TextConverterService Word;

void collectPaintOnBrush(KukaMoveit & manipulator, geometry_msgs::Pose & pose)
{
    geometry_msgs::Pose p;

    p.position.x = pose.position.x;
    p.position.y = pose.position.y;
    p.position.z = pose.position.z + COLOR_BOTLE_HEIGHT + HEIGHT_OFFSET + BRUSH_HEIGHT;
    p.orientation.w = 1;
    manipulator.move(p, DEBUG);

    p.position.z = pose.position.z + COLOR_HEIGHT + BRUSH_HEIGHT;
    manipulator.move(p, DEBUG);

    p.position.z = pose.position.z + COLOR_BOTLE_HEIGHT + HEIGHT_OFFSET + BRUSH_HEIGHT;
    manipulator.move(p, DEBUG);
}


void doSmear(KukaMoveit & manipulator, std::vector<geometry_msgs::Pose> & waypoints)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    geometry_msgs::Pose first, last;

    first = waypoints.front();
    first.position.z += HEIGHT_OFFSET;
    last = waypoints.back();
    last.position.z += COLOR_BOTLE_HEIGHT + HEIGHT_OFFSET;


    ROS_INFO_STREAM("P: [" << first.position.x << ", " << first.position.y << ", " << first.position.z << "]");

    // Move to start position
    manipulator.move(first, DEBUG);

    // Modify and execute waypoints
    // Set slow motion
    manipulator.getMoveGroup()->setMaxVelocityScalingFactor(0.8);
    double fraction = manipulator.getMoveGroup()->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory
    );
    for (size_t i = 0; i < trajectory.joint_trajectory.points.size(); ++i) {
        std::cout << "(" << i << ")\t" << trajectory.joint_trajectory.points[i].time_from_start << " | "
            << trajectory.joint_trajectory.points[i].positions[0] << std::endl;
    }
    plan.trajectory_ = trajectory;
    manipulator.getMoveGroup()->execute(plan);

    // Complete of trajectory
    manipulator.move(last, DEBUG);

    manipulator.getMoveGroup()->setMaxVelocityScalingFactor(1.2);

}

void transformFromCanvasToBase(std::vector<geometry_msgs::Pose> & poses) {

    // Transform:
    // R*p + v
    // R - canvas transform matrix, v - translation or canvas frame, p - painting point

    tf2::Vector3 result, p;
    for (size_t i = 0; i < poses.size(); ++i) {
        p = tf2::Vector3(
            poses[i].position.x,
            poses[i].position.y,
            poses[i].position.z
        );
        result = R*p + v;

        poses[i].position.x = result.m_floats[0];
        poses[i].position.y = result.m_floats[1];
        poses[i].position.z = result.m_floats[2];
    }
}

std::vector<geometry_msgs::PoseArray> readBagFile(const std::string filepath, const std::string topicName)
{
    rosbag::Bag bag;
    bag.open(filepath, rosbag::bagmode::Read);

    rosbag::View view(bag, rosbag::TopicQuery(topicName));
    std::vector<geometry_msgs::PoseArray> trajectoryArray;
    geometry_msgs::PoseArray p;

    foreach (rosbag::MessageInstance const m, view) {
        geometry_msgs::PoseArray::ConstPtr parr = m.instantiate<geometry_msgs::PoseArray>();

        p.poses = parr->poses;
        trajectoryArray.push_back(p);
    }

    bag.close();

    return trajectoryArray;
}

void chatterCallback(const std_msgs::String msg)
{
    ROS_INFO("I heard: [%s]", msg.data.c_str());
    Word.request.data = msg.data;
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
    ros::ServiceClient startImgProcClient = nh.serviceClient<local_task_planner::TextConverterService>("/convert_text");
    ros::Publisher start_pub = nh.advertise<std_msgs::String>("film", 10);

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

    while(ros::ok()) {
        if (start) {

	    /*send info string to server*/
	    startImgProcClient.call(Word);


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

            if (START_POSE_DEBUG) {
                ROS_INFO("[LTP] START_POSE_DEBUG --------------");
                double offset = 0.02;
                geometry_msgs::Pose c;
                std::vector<geometry_msgs::Pose> p;
                std::vector<tf2::Vector3> vp(4);
                p.resize(4);

                vp[0] = R*tf2::Vector3( canvasInfo.response.height/2, -canvasInfo.response.width/2, 0),
                vp[1] = R*tf2::Vector3(-canvasInfo.response.height/2, -canvasInfo.response.width/2, 0),
                vp[2] = R*tf2::Vector3(-canvasInfo.response.height/2,  canvasInfo.response.width/2, 0),
                vp[3] = R*tf2::Vector3( canvasInfo.response.height/2,  canvasInfo.response.width/2, 0),

                // Fille center of paper
                c.position.x = v.m_floats[0];
                c.position.y = v.m_floats[1];
                c.position.z = v.m_floats[2];

                // Move to center point
                manipulator.move(c, DEBUG);

                // p = center + R*vp
                for (size_t i = 0; i < 4; ++i) {
                    p[i].position.x = c.position.x + vp[i].m_floats[0];
                    p[i].position.y = c.position.y + vp[i].m_floats[1];
                    p[i].position.z = c.position.z + vp[i].m_floats[2] + offset;

                    // move between points
                    manipulator.move(p[i], DEBUG);
                }

            }

            /* Image palette info */
            std_srvs::Empty emptyMsg;
            ROS_INFO_STREAM("[LTP] START image processing.");



            /*if (!startImgProcClient.call(emptyMsg)) {
                ROS_ERROR_STREAM("\t ERROR");
                return 0;
            }*/


            // TODO Add rosbag file read function
            std::vector<geometry_msgs::PoseArray> trajectorys = readBagFile(BAG_FILE_PATH, TRAJECTORY_TOPIC_NAME);

            // Draw Params
            bool isDraw = true;
	    msg.data = "start";
	    start_pub.publish(msg);

            size_t numberOfSmears = trajectorys.size();
            size_t currentSmearNumber = 0;

            // Convert kuka_cv::Pose to geometry_msgs::Pose
            geometry_msgs::Pose colorPose;
            colorPose.position.x = palette.poses[0].x;
            colorPose.position.y = palette.poses[0].y;
            colorPose.position.z = palette.poses[0].z;

            ROS_INFO_STREAM("Drawing inforation: ");
            ROS_INFO_STREAM("Trajectorys number: " << numberOfSmears);

            // Global drawing circle
            collectPaintOnBrush(manipulator, colorPose);
            while (ros::ok() && isDraw) {
                if (currentSmearNumber == numberOfSmears) {
                //if (currentSmearNumber == numberOfSmears) {
                    ROS_ERROR("Drawing image complete");
                    isDraw = false;
		    msg.data = "stop";
            	    start_pub.publish(msg);
                    break;
                }

                transformFromCanvasToBase(trajectorys[currentSmearNumber].poses);
                doSmear(manipulator, trajectorys[currentSmearNumber].poses);
                collectPaintOnBrush(manipulator, colorPose) ;

                ROS_INFO_STREAM("[LTP] POINT (" << currentSmearNumber << ")");
                ++currentSmearNumber;
            }

           // std_srvs::Empty empty;
           // if (!liabilityFinishClient.call(empty)) {
           //     return 0;
           // }
            start = false;
        }
    }
    return 0;
}
