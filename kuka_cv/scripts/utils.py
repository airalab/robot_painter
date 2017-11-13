import rospy
import tf2_ros
import tf

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
def getCameraFrame():
    tempRate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("base_link", "camera_link", rospy.Time())
            print(trans)
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            tempRate.sleep()
            continue

    cameraPosition = trans.transform.translation;
    cameraRotation = trans.transform.rotation;

    rotMatrix = tf.transformations.quaternion_matrix([cameraRotation.x,
        cameraRotation.y,
        cameraRotation.z,
        cameraRotation.w])

    return cameraPosition, rotMatrix