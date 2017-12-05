#!/usr/bin/env python2

import roslib
import rospy
import tf
import tf2_ros

import geometry_msgs.msg
import tf2_msgs.msg

from kuka_cv.srv import *

if __name__ == '__main__':
	rospy.init_node("canvas_transform_broadcaster")

	canvasCient = rospy.ServiceProxy('/request_canvas', RequestCanvas)
	rospy.wait_for_service('/request_canvas')

	print("Waiting the infomation")
	canvasInfo = canvasCient()	

	br = tf2_ros.StaticTransformBroadcaster()
	t = geometry_msgs.msg.TransformStamped()

	t.header.frame_id = "/base_link"
	t.child_frame_id = "/canvas_link"
	t.transform = canvasInfo.trans;

	rate = rospy.Rate(10.0)
	print("Publishing the /canvas_link frame")
	while not rospy.is_shutdown():
		t.header.stamp = rospy.Time.now()
		br.sendTransform(t)
		rate.sleep()
