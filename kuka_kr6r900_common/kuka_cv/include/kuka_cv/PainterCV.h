#ifndef PAINTER_CV
#define PAINTER_CV

#include <CommonCV.h>

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <kuka_cv/Color.h>
#include <kuka_cv/Pose.h>
#include <kuka_cv/RequestCanvas.h>
#include <kuka_cv/RequestPalette.h>

// Boost
#include <boost/thread/thread.hpp>

// Librealsense
#include <librealsense/rs.hpp>

struct CameraWorkingInfo
{
    // 1 - from image;
    // 2 - using onboard web camera
    // 3 - using intel realsense
    // 4 - work with topic
    int workMode;

    // If selected mode 1
    std::string imagePath;

    // If selected mode 2
    int deviceNumber;

    // If selected mode 4
    std::string topicName;
    std::string messageType;

    // If selected mode 3
    // Realsense automaticly find device numer
    // and other information

    // Other information
    // If kx and ky are unnown set kx and ky = 1;
    double kx, ky;
};

class PainterCV
{
    public:
        PainterCV(ros::NodeHandle & nh, CameraWorkingInfo info, int frequency);
        ~PainterCV();

        void getImage(cv::Mat & img);

        /// Image processing
        Quadrilateral detectCanvas(cv::Mat & src);
        void detectPaletteColors(cv::Mat & src,
            std::vector<cv::Point> & p, std::vector<cv::Vec3b> & c);
        void loadServices();

        kx = info.kx; ky = info.ky;
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

        /// Tranform frame from default to join 6
        void transformCameraFrame(kuka_cv::Pose & p);

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