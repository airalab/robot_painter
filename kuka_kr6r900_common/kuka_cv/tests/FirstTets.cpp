#include <CommonCV.h>

// include the librealsense C++ header file
#include <librealsense/rs.hpp>

// 1 - test
// 2 - onboard
// 3 - realsense
#define CAMERA 3
#define CAMERA_NUM 0 // Only for onboard cameras!
std::string imageName("/home/senex/Tests/RealsenseCV/SavedImages/Image.jpg");


rs::device * findCamera(rs::context & ctx) {
    int device_count = ctx.get_device_count();
    if (!device_count) printf("No device detected. Is it plugged in?\n");
    rs::device * dev;

    for(int i = 0; i < device_count; ++i) {
        // Show the device name and information
        dev = ctx.get_device(i);
        std::cout << "Device " << i << " - " << dev->get_name() << ":\n";
        std::cout << " Serial number: " << dev->get_serial() << "\n";
        std::cout << " Firmware version: " << dev->get_firmware_version() << "\n";
        try { std::cout << " USB Port ID: " << dev->get_usb_port_id() << "\n"; } catch (...) {}
        if (dev->supports(rs::capabilities::adapter_board)) std::cout << " Adapter Board Firmware version: " << dev->get_info(rs::camera_info::adapter_board_firmware_version) << "\n";
        if (dev->supports(rs::capabilities::motion_events)) std::cout << " Motion Module Firmware version: " << dev->get_info(rs::camera_info::motion_module_firmware_version) << "\n";
    }

    return dev;
}

int main(int argc, char ** argv)
{
    const auto window_name = "Image";
    cv::Mat image;
    std::vector<Quadrilateral> quad;
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

#if CAMERA == 1
    image = cv::imread(imageName, 1);
    if (image.empty()) {
        std::cout << "Image is empty" << std::endl;
        return 1;
    }
    quad = findQuadrilateralByHough(image);
    for (size_t q = 0; q < quad.size(); ++q) {
        drawQuadrilateral(image, quad[q]);
    }
    findCirclesByCanny(image);
    cv::imshow(window_name, image);
    cv::waitKey(0);
#elif CAMERA == 2
    cv::VideoCapture videoCapture;
    videoCapture.open(CAMERA_NUM);
    if (!videoCapture.isOpened()) {
        std::cout << "Could not open reference " << 0 << std::endl;
        return -1;
    }
    while(cv::waitKey(1) < 0 && cvGetWindowHandle(window_name)) {
        videoCapture >> image;
        quad = findQuadrilateralByHough(image);
        for (size_t q = 0; q < quad.size(); ++q) {
            drawQuadrilateral(image, quad[q]);
        }
        cv::imshow(window_name, image);
    }
#elif CAMERA == 3
    // Obtain a list of devices currently present on the system
    // Create a context object. This object owns the handles to all connected realsense devices
    rs::context ctx;
    rs::device * dev = findCamera(ctx);
    // Configure Infrared streaming to run at VGA resolution at 30 frames per second
    dev->enable_stream(rs::stream::color, 640, 480, rs::format::bgr8, 30);

    // Start streaming
    dev->start();
    for (size_t i = 0; i < 30; ++i)
        dev->wait_for_frames();

    while(cv::waitKey(1) < 0 && cvGetWindowHandle(window_name)) {
        // Camera warmup - Dropped several first frames to let auto-exposure stabilize
        dev->wait_for_frames();
        // Creating OpenCV Matrix from a color image
        image = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)dev->get_frame_data(rs::stream::color), cv::Mat::AUTO_STEP);
    }

#endif

    return 0;
}