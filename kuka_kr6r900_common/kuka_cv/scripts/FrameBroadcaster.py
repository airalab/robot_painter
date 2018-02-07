#!/usr/bin/env python2

import roslib
import rospy
import tf
import tf2_ros

import math

import geometry_msgs.msg
import tf2_msgs.msg

if __name__ == '__main__':
	rospy.init_node("frame_broadcaster")

	print("Starting frame broadcaster.")

	cameraPos = rospy.get_param("/frame_broadcaster/camera_positon")
	brushPos = rospy.get_param("/frame_broadcaster/brush_position")

	br = tf2_ros.StaticTransformBroadcaster()

	t1 = geometry_msgs.msg.TransformStamped()
	t1.header.frame_id = "/link_6"
	t1.child_frame_id = "/camera_link"
	t1.transform.translation.x = cameraPos[0]
	t1.transform.translation.y = cameraPos[1]
	t1.transform.translation.z = cameraPos[2]
	q = tf.transformations.quaternion_from_euler(0, 0, 0)
	t1.transform.rotation.x = q[0]
	t1.transform.rotation.y = q[1]
	t1.transform.rotation.z = q[2]
	t1.transform.rotation.w = q[3]

	t2 = geometry_msgs.msg.TransformStamped()
	t2.header.frame_id = "/link_6"
	t2.child_frame_id = "/brush_link"
	t2.transform.translation.x = brushPos[0]
	t2.transform.translation.y = brushPos[1]
	t2.transform.translation.z = brushPos[2]
	q2 = tf.transformations.quaternion_from_euler(math.pi, 0, 0)
	t2.transform.rotation.x = q2[0]
	t2.transform.rotation.y = q2[1]
	t2.transform.rotation.z = q2[2]
	t2.transform.rotation.w = q2[3]

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		t1.header.stamp = rospy.Time.now()
		t2.header.stamp = rospy.Time.now()
		br.sendTransform(t1)
		br.sendTransform(t2)
		rate.sleep()