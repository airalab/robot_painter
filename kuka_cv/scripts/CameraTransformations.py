import rospy
import tf2_ros
import tf

class CameraTransformations:

    def __init__(self, resolution, scaleFactor, baseFrameName, cameraFrameName):

        self.baseLink = baseFrameName
        self.cameraLink = cameraFrameName
        self.width = resolution[0]          # camera resolution [px]
        self.height = resolution[1]         # camera resolution [px]
        self.planeVecX = 0                  # X coordinate of plane normal
        self.planeVecY = 0                  # Y coordinate of plane normal
        self.planeVecZ = 0                  # Z coordinate of plane normal
        self.dx = 0                         # camera in manipulator frame [m]
        self.dy = 0                         # camera in manipulator frame [m]
        self.dz = 0                         # camera in manipulator frame [m]
        self.rot = 0                        # camera Transformation 4x4 matrix
        self.q = 0                          # quaternion
        self.kx = scaleFactor[0]            # scale factor [m/px]
        self.ky = scaleFactor[1]            # scale factor [m/px]
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def setVectorFromCameraToPlane(self, objectVector):
        self.planeVecX = objectVector[0]
        self.planeVecY = objectVector[1]
        self.planeVecZ = objectVector[2]

    def coordTransform(self, x, y, z):
        # First transform
        # Rotate camera frame to frame of joint_a6 (link_6)
        x1 = -(y - self.height/2)*self.kx + self.planeVecX
        y1 = (x - self.width/2)*self.ky + self.planeVecY
        z1 = z + self.planeVecZ

        # print("p: " + str((x1, y1, z1)))
        # Second transform
        # Move to vector [dx, dy, dz]
        # Rotate camera frame to base frame (base_link)
        newX = self.rot[0, 0]*x1 + self.rot[0, 1]*y1 + self.rot[0, 2]*z1 + self.dx
        newY = self.rot[1, 0]*x1 + self.rot[1, 1]*y1 + self.rot[1, 2]*z1 + self.dy
        newZ = self.rot[2, 0]*x1 + self.rot[2, 1]*y1 + self.rot[2, 2]*z1

        return [newX, newY, newZ, x1, y1, z1]

    def getCameraFrame(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform(self.baseLink, self.cameraLink, rospy.Time())
                print(trans)
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

        pos = trans.transform.translation
        q = trans.transform.rotation
        self.q = [q.x, q.y, q.z, q.w]

        self.dx = pos.x
        self.dy = pos.y
        self.dz = pos.z
        self.rot = tf.transformations.quaternion_matrix(self.q)

