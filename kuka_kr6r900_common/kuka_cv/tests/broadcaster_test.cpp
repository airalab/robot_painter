#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "br_test");
    ros::NodeHandle nh;

    static tf2_ros::TransformBroadcaster tf2_broadcaster;
    geometry_msgs::TransformStamped transform;

    transform.header.frame_id = "link_6";
    transform.child_frame_id  = "test_link";
    transform.transform.translation.x = 0.1;
    transform.transform.translation.y = 0.1;
    transform.transform.translation.z = 0.1;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;


    while (ros::ok()) {
        transform.header.stamp = ros::Time::now();

        tf2_broadcaster.sendTransform(transform);
        ROS_INFO_STREAM("Send Transform");
        ROS_INFO_STREAM("t: \n" << transform);
        ros::Duration(1).sleep();
    }

    return 0;
}