import rospy
import tf2_ros
import tf

class CameraTransformations:

    def __init__(self, resolution, scaleFactor, baseFrameName, cameraFrameName):

        self.baseLink = baseFrameName
        self.cameraLink = cameraFrameName
        self.width = resolution[0]          # camera resolution [px]
        self.height = resolution[1]         # camera resolution [px]
        self.objx = 0                       # X coordinate of plane normal
        self.objy = 0                       # Y coordinate of plane normal
        self.objz = 0                       # Z coordinate of plane normal
        self.dx = 0                         # camera in manipulator frame [m]
        self.dy = 0                         # camera in manipulator frame [m]
        self.dz = 0                         # camera in manipulator frame [m]
        self.rot = 0                        # camera Transformation 4x4 matrix
        self.kx = scaleFactor[0]            # scale factor [m/px]
        self.ky = scaleFactor[1]            # scale factor [m/px]
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(tfBuffer)

        self.paleteMsg = Palette()

    def setObjectVector(self, objectVector):
        self.objx = objectVector[0]
        self.objy = objectVector[1]
        self.objz = objectVector[2]

    def coordTransform(self, x, y, z):
        # First transform
        # Rotate camera frame to frame of joint_a6 (link_6)
        x1 = -(y - self.height/2)*self.kx + objx
        y1 = (x - self.width/2)*self.ky + objy
        z1 = objz

        # print("p: " + str((x1, y1, z1)))
        # Second transform
        # Move to vector [dx, dy, dz]
        # Rotate camera frame to base frame (base_link)
        newX = self.rot[0, 0]*x1 + self.rot[0, 1]*y1 + self.rot[0, 2]*z1 + self.dx
        newY = self.rot[1, 0]*x1 + self.rot[1, 1]*y1 + self.rot[1, 2]*z1 + self.dy
        newZ = self.rot[2, 0]*x1 + self.rot[2, 1]*y1 + self.rot[2, 2]*z1

        return [round(newX, 4), round(newY, 4), round(newZ, 4)]

    def getCameraFrame(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform(baseFrameName, cameraFrameName, rospy.Time())
                print(trans)
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

        cameraPosition = trans.transform.translation;
        cameraRotation = trans.transform.rotation;

        self.dx = cameraPosition.x
        self.dy = cameraPosition.y
        self.dz = cameraPosition.z
        self.rot = tf.transformations.quaternion_matrix([cameraRotation.x,
            cameraRotation.y,
            cameraRotation.z,
            cameraRotation.w])

