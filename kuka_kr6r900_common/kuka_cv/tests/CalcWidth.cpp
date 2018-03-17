#include <CommonCV.h>

#define TEST false
std::string imageName("/home/senex/Tests/RealsenseCV/SavedImages/Image.jpg");

int main(int argc, char ** argv)
{
    const auto window_name = "Image";
    cv::Mat image;
    std::vector<Quadrilateral> quad;
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

#if TEST
    image = cv::imread(imageName, 1);
    if (image.empty()) {
        std::cout << "Image is empty" << std::endl;
        return 1;
    }
    quad = findQuadrilateralByHough(image);
    for (size_t q = 0; q < quad.size(); ++q) {
        drawQuadrilateral(image, quad[q]);
        cv::Point diff1 = quad[q].p[0] - quad[q].p[1];
        cv::Point diff2 = quad[q].p[1] - quad[q].p[2];
        cv::putText(image, std::to_string((int)sqrt(diff1.x*diff1.x + diff1.y*diff1.y)), quad[q].center, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,200), 2, CV_AA);
        cv::putText(image, std::to_string((int)sqrt(diff2.x*diff2.x + diff2.y*diff2.y)), quad[q].center + cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,200), 2, CV_AA);
    }
    cv::imshow(window_name, image);
    cv::waitKey(0);
#else 
    int cameraNum = 0;
    cv::VideoCapture videoCapture;
    videoCapture.open(cameraNum);
    if (!videoCapture.isOpened()) {
        std::cout << "Could not open reference " << 0 << std::endl;
        return -1;
    }
    while(cv::waitKey(1) < 0 && cvGetWindowHandle(window_name)) {
        videoCapture >> image;
        quad = findQuadrilateralByHough(image);
        for (size_t q = 0; q < quad.size(); ++q) {
            drawQuadrilateral(image, quad[q]);
            cv::Point diff1 = quad[q].p[0] - quad[q].p[1];
            cv::Point diff2 = quad[q].p[1] - quad[q].p[2];
            cv::putText(image, std::to_string((int)sqrt(diff1.x*diff1.x + diff1.y*diff1.y)), quad[q].center, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,200), 2, CV_AA);
            cv::putText(image, std::to_string((int)sqrt(diff2.x*diff2.x + diff2.y*diff2.y)), quad[q].center + cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,200), 2, CV_AA);
        }
        cv::imshow(window_name, image);
    }
#endif

    return 0;
}