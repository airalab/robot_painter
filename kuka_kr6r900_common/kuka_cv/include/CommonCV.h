#ifndef COMMON_CV
#define COMMON_CV
// include OpenCV header file
#include <opencv2/opencv.hpp>

#define CANNY_THRESK 80
#define MIN_AREA 700
#define BLUR_KERNEL 13

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

    // TF camera link name
    std::string cameraLinkName;

    // Brush transform
    double brushTransform;
};

struct Quadrilateral
{

public:
    Quadrilateral()
    {
        p.resize(4);
        center = cv::Point(0, 0);
    }
    Quadrilateral(cv::Point pt1, cv::Point pt2, cv::Point pt3, cv::Point pt4)
    {
        p.resize(4); p[0] = pt1; p[1] = pt2; p[2] = pt3; p[3] = pt4;
        calcCenter();
    }
    Quadrilateral& operator=(const Quadrilateral& q) {
        this->p = q.p;
        this->center = q.center;
        return *this;
    }

    std::vector<cv::Point> p;
    cv::Point center;
private:
    void calcCenter()
    {
        int A1 = (p[2].y - p[0].y), B1 = -(p[2].x - p[0].x), C1 = p[0].x*p[2].y - p[2].x*p[0].y;
        int A2 = (p[3].y - p[1].y), B2 = -(p[3].x - p[1].x), C2 = p[1].x*p[3].y - p[3].x*p[1].y;
        int det = A1*B2 - A2*B1;
        center.x = ( B2*C1 - B1*C2)/det;
        center.y = (-A2*C1 + A1*C2)/det;
    }
};

bool checkUniqueOfVector(std::vector<cv::Vec2f> & vec, cv::Vec2f critria, cv::Vec2f maxDiff);
bool checkUniqueOfVector(std::vector<Quadrilateral> & vec, Quadrilateral critria, cv::Vec2f maxDiff);
void drawQuadrilateral(cv::Mat & image, Quadrilateral & q);
std::vector<Quadrilateral> findQuadrilateralByHough(cv::Mat & src);
void findCirclesByCanny(cv::Mat & src, std::vector<float> & circlesRadii, std::vector<cv::Point> & circlesCenters);

// TODO add trackbars
#endif