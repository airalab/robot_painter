#include <CommonCV.h>
#include <ros/ros.h>
#include <ros/package.h>


   	// std::cout << << std::endl;
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "canvas_test");
    ros::NodeHandle nh;

    Quadrilateral q;
    std::vector<Quadrilateral> quads;

    std::string path = ros::package::getPath("kuka_cv");
    cv::Mat image = cv::imread(path + "/images/error_canvas_image.jpg", CV_LOAD_IMAGE_COLOR);

    cv::namedWindow("Image", 1);
    quads = findQuadrilateralByHough(image);
   	std::cout << "Quads size: " << quads.size() << std::endl;
   	if (quads.size() == 1)
	   	drawQuadrilateral(image, quads[0]);

    cv::imshow("Image", image);
    cv::waitKey(0);

    return 0;
}
