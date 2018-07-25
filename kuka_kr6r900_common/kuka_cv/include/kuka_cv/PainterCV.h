#ifndef PAINTER_CV
#define PAINTER_CV

#include <CommonCV.h>
#include <CameraTransforms.h>

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/TransformStamped.h>

#include <kuka_cv/Color.h>
#include <kuka_cv/Pose.h>
#include <kuka_cv/RequestCanvas.h>
#include <kuka_cv/RequestPalette.h>

// Boost
#include <boost/thread/thread.hpp>

// Librealsense
#include <librealsense/rs.hpp>

class PainterCV
{
    public:
        PainterCV(ros::NodeHandle & nh, CameraWorkingInfo info, int frequency);
        ~PainterCV();

        cv::Mat getImage();

        /// Image processing
        Quadrilateral detectCanvas(cv::Mat & src);
        bool detectPaletteColors(cv::Mat & src,
            std::vector<cv::Point> & p, std::vector<cv::Vec3b> & c);
        void loadServices();

        CameraTransforms camTransforms;

    private:

        /// Image updaters
        /// The frequency of working updater is equal to freq
        void updateImageByOpenCV();
        void updateImageByLibrealsense();
        void imageCallback(const sensor_msgs::Image::ConstPtr & msg);

        /// Detecting Callbacs
        bool canvasCallback(kuka_cv::RequestCanvas::Request  & req,
                            kuka_cv::RequestCanvas::Response & res);
        bool paletteCallback(kuka_cv::RequestPalette::Request  & req,
                             kuka_cv::RequestPalette::Response & res);

        /// get Transform vector from base_link to camera_link
        void getTransformFromBaseToCam(kuka_cv::Pose & p);

        ros::ServiceServer canvasServer;
        ros::ServiceServer paletteService;

        ros::Subscriber cameraSubscriber;

        /// Main circle frequency in hz
        int freq;

        /// Same as CameraWorkingInfo work mode
        int workMode;

        /// Actual image from camera
        cv::Mat image;

        /// Actual device number of camera
        int devNum;

        /// thread
        boost::thread thr;

        /// CvBridge image
        cv_bridge::CvImagePtr cv_ptr;

        /// Camera resolution
        double camWidth, camHeight;
        /// Camera m/px coefficients
        double kx, ky;

        /// Main information
        kuka_cv::RequestPalette::Response palette;
        kuka_cv::RequestCanvas::Response canvas;

        /// Global Node handle
        ros::NodeHandle n;

        std::string packagePath;
};

rs::device * findCamera(rs::context & ctx);

#endif