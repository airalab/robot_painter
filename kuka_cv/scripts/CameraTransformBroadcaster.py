#!/usr/bin/env python2

import roslib
import rospy
import tf
import tf2_ros

import geometry_msgs.msg

rospy.init_node("camera_transform_broadcaster")

cameraPos = rospy.get_param("/camera_transform_broadcaster/camera_positon")
x, y, z = cameraPos[0], cameraPos[1], cameraPos[2]
print("Camera position: " + str(x) + ", " 
	+ str(y) + ", " + str(z))

br = tf2_ros.TransformBroadcaster()
t = geometry_msgs.msg.TransformStamped()

t.header.stamp = rospy.Time.now()
t.header.frame_id = "base_link"
t.child_frame_id = "camera_link"
t.transform.translation.x = x
t.transform.translation.y = y
t.transform.translation.z = z
q = tf.transformations.quaternion_from_euler(0, 0, 0)
t.transform.rotation.x = q[0]
t.transform.rotation.y = q[1]
t.transform.rotation.z = q[2]
t.transform.rotation.w = q[3]

while not rospy.is_shutdown():
	br.sendTransform(t)