#include <kuka_cv/PainterCV.h>
#include <ros/package.h>
#define TEST_Z 0.85

void showPalette(cv::Mat image, std::vector<cv::Point> & p, std::vector<kuka_cv::Pose> & pTransformed) {

    ROS_INFO_STREAM("[CV] PShow palette");

    std::string windowName = "circles";
    const char name[] = "circles";
    int height = image.rows;
    int width = image.cols;
    cv::namedWindow(name);

    ///// Default axis
    //
    //     ─┼───────────━━ x
    //      │
    //      │
    //      │
    //      │
    //      ┃
    //      y

    // x axis
    cv::arrowedLine(image, cv::Point(width/2, 0), cv::Point(width/2, height), cv::Scalar(255, 0, 0), 2, 8, 0, 0.01);
    cv::putText(image, cv::String("X"), cv::Point(width/2 + 20, height), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0, 0, 0), 2);

    // y axis
    cv::arrowedLine(image, cv::Point(0, height/2), cv::Point(width, height/2), cv::Scalar(255, 0, 0), 2, 8, 0, 0.01);
    cv::putText(image, cv::String("Y"), cv::Point(width, height/2 + 20), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0, 0, 0), 2);

    // Colors coordinates
    std::stringstream ss;
    for (size_t i = 0; i < p.size(); ++i) {
        ss << "(" << pTransformed[i].x << ", " << pTransformed[i].y << ")";
        cv::putText(image, cv::String(ss.str()), p[i], cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0, 0, 0), 2);
        ss.str("");
    }

    // Draw image
    // TODO Add support of 'x' button for window with cvGetWindowHandle
    cv::imshow(name, image);
    while (cv::waitKey(1) < 0 && cvGetWindowHandle(name) && ros::ok()) {
        ROS_INFO_STREAM("[Painter CV] Please check the image and press button.");
        ros::Duration(1).sleep();
    }
    cv::destroyWindow(name);
}


PainterCV::PainterCV(ros::NodeHandle & nh, CameraWorkingInfo info, int frequency)
: n(nh), camTransforms(nh, info)
{
    freq = frequency;
    workMode = info.workMode;
    camWidth = 0; camHeight = 0;
    packagePath = ros::package::getPath("kuka_cv");
    kx = info.kx; ky = info.ky;

    switch(info.workMode) {
        case 1: // From image
            image = cv::imread(info.imagePath, CV_LOAD_IMAGE_COLOR);
            // TODO do image processing
            cv::imshow("image", image);
            cv::waitKey(0);
            break;
        case 2: // Onboard web camera
            devNum = info.deviceNumber;
            std::cout << "[Painter CV] Start Web camera " << devNum << std::endl;
            thr = boost::thread(&PainterCV::updateImageByOpenCV, this);
            break;
        case 3: // Intel Realsense
            camWidth = 640; camHeight = 480;
            camTransforms.setCameraDimensions(camWidth, camHeight);
            std::cout << "[Painter CV] Start Inter Realsense " << std::endl;
            thr = boost::thread(& PainterCV::updateImageByLibrealsense, this);
            break;
        case 4: // Topic
            std::cout << "[Painter CV] Start topic Subscriber: " << info.topicName << std::endl;
            cameraSubscriber = nh.subscribe(info.topicName, 10, &PainterCV::imageCallback, this);
            break;
        default:
            std::cout << "[Painter CV] Work Mode is not correct!" << std::endl;
    }
}
PainterCV::~PainterCV()
{}

void PainterCV::loadServices()
{
    canvasServer = n.advertiseService("request_canvas", &PainterCV::canvasCallback, this);
    paletteService = n.advertiseService("request_palette", &PainterCV::paletteCallback, this);
}
void PainterCV::updateImageByOpenCV()
{
    cv::VideoCapture videoCapture;
    videoCapture.open(0);
    if (!videoCapture.isOpened()) {
        std::cout << "Could not open reference " << 0 << std::endl;
        return;
    }

    videoCapture >> image;
    if (camWidth == 0 || camHeight == 0) {
        camWidth = image.cols; camHeight = image.rows;
        camTransforms.setCameraDimensions(camWidth, camHeight);
    }

    ros::Rate rate(freq);
    while(ros::ok()) {
        videoCapture >> image;
        rate.sleep();
    }
}
void PainterCV::updateImageByLibrealsense()
{
    // Obtain a list of devices currently present on the system
    // Create a context object. This object owns the handles to all connected realsense devices
    rs::context ctx;
    int device_count = ctx.get_device_count();
    if (!device_count) {
        ROS_ERROR("No device detected. Is it plugged in?\n");
        ros::shutdown();
        return;
    }
    rs::device * dev = findCamera(ctx);

    // Configure Infrared streaming to run at VGA resolution at 30 frames per second
    dev->enable_stream(rs::stream::color, camWidth, camHeight, rs::format::bgr8, 30);

    // Start streaming
    dev->start();

    ros::Rate rate(freq);
    while(ros::ok()) {
        // Camera warmup - Dropped several first frames to let auto-exposure stabilize
        dev->wait_for_frames();
        // Creating OpenCV Matrix from a color image
        image = cv::Mat(cv::Size(camWidth, camHeight), CV_8UC3, (void*)dev->get_frame_data(rs::stream::color), cv::Mat::AUTO_STEP);

        rate.sleep();
    }
}
void PainterCV::imageCallback(const sensor_msgs::Image::ConstPtr & msg)
{

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        ros::shutdown();
    }

    image = cv_ptr->image;
    if (camWidth == 0 || camHeight == 0) {
        camWidth = image.cols; camHeight = image.rows;
        camTransforms.setCameraDimensions(camWidth, camHeight);
    }
    ros::Duration(round(1/freq)).sleep();
}

cv::Mat PainterCV::getImage()
{
    return image;
}

// Quadrilateral PainterCV::detectCanvas(cv::Mat & src)
// {
//     std::vector<Quadrilateral> quad;

//     quad = findQuadrilateralByHough(src);

//     // drawQuadrilateral(src, quad[0]);
//     // cv::imshow("Canvas", src);
//     // cv::waitKey(0);
//     return quad[0];
// }
bool PainterCV::detectPaletteColors(cv::Mat & src, std::vector<cv::Point> & p, std::vector<cv::Vec3b> & c)
{
    std::vector<float> radii;
    findCirclesByCanny(src, radii, p);
    if (p.empty()) {
        return false;
        // ros::shutdown();
    }
    c.resize(p.size());
    for (size_t i = 0; i < p.size(); ++i) {
        // std::cout << "[radius " << i << "] \t" << radii[i] << std::endl;
        c[i] = src.at<cv::Vec3b>(p[i]);
        // std::cout << "ColoR: (" <<
        // static_cast<unsigned int>(src.at<cv::Vec3b>(p[i])[0]) << ", " <<
        // static_cast<unsigned int>(src.at<cv::Vec3b>(p[i])[1]) << ", " <<
        // static_cast<unsigned int>(src.at<cv::Vec3b>(p[i])[2]) << ")" << std::endl;
    }
    // cv::imshow("circles", src);
    // cv::waitKey(0);
    return true;
}

////////////////////// Callbacks
bool PainterCV::canvasCallback(kuka_cv::RequestCanvas::Request  & req,
                               kuka_cv::RequestCanvas::Response & res)
{
    // Modes:
    // 0 - measure
    // 1 - get info

    if (req.mode == 0) {
        ROS_INFO("Start Image Processing | Try to find canvas ...");
        std::vector<Quadrilateral> q;
        Quadrilateral quad;
        double width, height;
        kuka_cv::Pose p;
        cv::Point diff1;
        cv::Point diff2;

        // Update camera frame
        camTransforms.updateFrame();

        // Find all quadraterials
        size_t i = 0;
        while (q.size() != 1 && i < 100) {
            q = findQuadrilateralByHough(image);
            ++i;
        }
        if (i > 99) {
            ROS_ERROR("Canvas not found!!!");
            cv::imwrite(packagePath + "/images/error_canvas_image.jpg", image);
            ros::shutdown();
            return false;
        }
        quad = q[0];

        diff1 = quad.p[0] - quad.p[1];
        diff2 = quad.p[1] - quad.p[2];

        // Fill canvas pose message
        p.x = quad.center.x;
        p.y = quad.center.y;
        p.z = TEST_Z;
        camTransforms.transformPoint(p);

        // Calculate with and height of canvas
        width = (sqrt(diff1.x * diff1.x * kx*kx + diff1.y * diff1.y * ky*ky));
        height = (sqrt(diff2.x * diff2.x * kx*kx + diff2.y * diff2.y * ky*ky));

        canvas.p = p;
        canvas.width = width;
        canvas.height = height;
        res = canvas;
    } else if (req.mode == 1) {
        res = canvas;
    }

    return true;
}
bool PainterCV::paletteCallback(kuka_cv::RequestPalette::Request  & req,
                                kuka_cv::RequestPalette::Response & res)
{
    // Modes:
    // 0 - measure
    // 1 - get info

    if (req.mode == 0) {
        std::vector<cv::Point> p;       // Palette position of center
        std::vector<kuka_cv::Pose> p1;
        std::vector<cv::Vec3b> c;       // Paints colors
        kuka_cv::Color color;
        kuka_cv::Pose pose;

        // Update camera frame
        camTransforms.updateFrame();

        // Recognize circle colors at palette
        if (!detectPaletteColors(image, p, c)) {
            ROS_ERROR("No circles was found!");
            return false;
        }

        ROS_INFO_STREAM("[CV] Palette size: " << p.size());
        palette.colors.resize(p.size());
        palette.poses.resize(p.size());
        p1.resize(p.size());
        for (size_t i = 0; i < p.size(); ++i) {
            color.r = c[i][0];
            color.g = c[i][1];
            color.b = c[i][2];
            pose.x =  p[i].x;
            pose.y =  p[i].y;
            pose.z =  TEST_Z;
            camTransforms.transformPoint(pose);

            palette.colors[i] = color;
            palette.poses[i] = pose;
            p1[i] = pose;
        }
        res = palette;
        showPalette(image, p, p1);
    } else if (req.mode == 1) {
        res = palette;
    }
    return true;
}

/// Utils
rs::device * findCamera(rs::context & ctx) {
    int device_count = ctx.get_device_count();
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