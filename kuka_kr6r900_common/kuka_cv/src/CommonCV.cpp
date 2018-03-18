#include <CommonCV.h>
#include <iostream>
#include <math.h>

bool checkUniqueOfVector(std::vector<cv::Vec2f> & vec, cv::Vec2f critria, cv::Vec2f maxDiff)
{
    if (vec.empty())
        return true;
    for (size_t i = 0; i < vec.size(); ++i)
        if (abs(vec[i][0] - critria[0]) < maxDiff[0] && abs(vec[i][1] - critria[1]) < maxDiff[1])
            return false;

    return true;
}

bool checkUniqueOfVector(std::vector<Quadrilateral> & vec, Quadrilateral critria, cv::Vec2f maxDiff)
{
    if (vec.empty())
        return true;
    for (size_t i = 0; i < vec.size(); ++i)
        if (abs(vec[i].center.x - critria.center.x) < maxDiff[0] && abs(vec[i].center.y - critria.center.y) < maxDiff[1])
            return false;

    return true;
}

std::vector<Quadrilateral> findQuadrilateralByHough(cv::Mat & src)
{
    cv::Mat blur, mask;

     /// Blur
    cv::medianBlur(src, blur, 9);
    // imshow("blur", blur);

    /// Edge detection
    cv::Canny(blur, mask, 60, 3*60, 3);
    // imshow("canny", mask);

    std::vector<Quadrilateral> rectangles;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    for(int i = 0; i < contours.size(); ++i) {
        if (cv::contourArea(contours[i]) > MIN_AREA) {
            cv::Mat drawing = cv::Mat::zeros(mask.size(), CV_8UC1);
            std::vector<cv::Vec2f> lines;
            std::vector<cv::Vec2f> uniLin;
            cv::drawContours(drawing, contours, i, cv::Scalar(255), 1, CV_AA, hierarchy, 0, cv::Point());
            cv::HoughLines(drawing, lines, 1, CV_PI/180, 100, 0, 0);

            for(size_t ln = 0; ln < lines.size(); ln++)
            {
                if (lines[ln][0] < 0) {
                    lines[ln][0] = abs(lines[ln][0]);
                    lines[ln][1] -= CV_PI;
                }
                if (checkUniqueOfVector(uniLin, lines[ln], cv::Vec2f(20, 0.1))) {
                    uniLin.push_back(lines[ln]);
                }
            }

            if (uniLin.size() != 4) continue;

            // Find Intersections
            std::vector<cv::Point> points;
            cv::Point pt;
            int ln1 = 0, ln1_prev = 0;;
            int ln2 = 0, ln2_check = 0;
            double det = 0;
            bool fullCircle = false;
            while(points.size() != 4) {
                det = 0;
                ln1_prev = ln1;
                ln1 = ln2;
                ln2 = 0;

                --ln2;
                do {
                    ++ln2;
                    if (ln2 == 4) ln2 = 0;
                    if (ln2 == ln1 || ln2 == ln1_prev) {
                        if (ln2 == 3) ln2 = 0;
                        else ++ln2;
                    }
                    if (ln2 == ln1 || ln2 == ln1_prev) {
                        if (ln2 == 3) ln2 = 0;
                        else ++ln2;
                    }
                    if (ln2_check != 0 && ln2 == ln2_check) {
                        fullCircle = true;
                        break;
                    }
                    if (ln2_check == 0) ln2_check == ln2;
                    det = sin(uniLin[ln2][1] - uniLin[ln1][1]); 
                } while (cv::abs(det) < 0.3);
                if (fullCircle) {
                    std::cout << "Error! Ln2 if full circle!" << std::endl;
                    break;
                }

                pt.x =  uniLin[ln1][0]*sin(uniLin[ln2][1])/det - uniLin[ln2][0]*sin(uniLin[ln1][1])/det;
                pt.y = -uniLin[ln1][0]*cos(uniLin[ln2][1])/det + uniLin[ln2][0]*cos(uniLin[ln1][1])/det;
                points.push_back(pt);
            } 
            rectangles.push_back(Quadrilateral(points[0], points[1], points[2], points[3]));
        }
    }
    std::vector<Quadrilateral> uniRect;
    for (size_t i = 0; i < rectangles.size(); ++i) {
        if(checkUniqueOfVector(uniRect, rectangles[i], cv::Vec2f(10, 10))) {
            uniRect.push_back(rectangles[i]);
            std::cout << "rect " << i << ": \t[" << uniRect[i].p[0] << ", " << uniRect[i].p[1] << ", " << uniRect[i].p[2] << ", " << uniRect[i].p[3] << "]" << std::endl;
            std::cout << "center " << i << ": \t" << uniRect[i].center << std::endl;
        }
    }
    return uniRect;
}
void drawQuadrilateral(cv::Mat & image, Quadrilateral & q) {
    for (size_t p = 0; p < 4; ++p) {
        if (p != 3)
            cv::line(image, q.p[p], q.p[p+1], cv::Scalar(0, 200, 200), 2, 8);
        else 
            cv::line(image, q.p[p], q.p[0], cv::Scalar(0, 200, 200), 2, 8);
        cv::circle(image, q.p[p], 5, cv::Scalar(0, 0, 200), 2, 8);
        cv::putText(image, std::to_string(p), q.p[p], cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0,0,200), 2, CV_AA);
    }
    cv::circle(image, q.center, 5, cv::Scalar(0, 0, 255), 2, 8);
}

void findCirclesByCanny(cv::Mat & src, std::vector<float> & circlesRadii, std::vector<cv::Point> & circlesCenters)
{
    cv::Mat dst, gray, edges;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> contoursPoly;
    std::vector<cv::Vec4i> hierarchy;
    float radius;
    cv::Point2f center;
    int ratio = 3;
    bool valid = false;
    int cannyKernel = 3;

    // 1. Bluring
    cv::GaussianBlur(src, dst, cv::Size(BLUR_KERNEL, BLUR_KERNEL), 0, 0);
    cv::cvtColor(dst, gray, CV_BGR2GRAY);

    // Optimal - 80
    cv::Canny(gray, edges, 80, 80*ratio, cannyKernel);

    /// Find contours
    cv::findContours(edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    contoursPoly.resize(contours.size());
    cv::Scalar color = cv::Scalar(0, 0, 255);

    /// Draw contours
    for (int i = 0; i < contours.size(); ++i) {
        cv::approxPolyDP(cv::Mat(contours[i]), contoursPoly[i], 0.1, true);

        // cout << contoursPoly[i].size() << endl;
        if ((contoursPoly[i].size() > 30) & (contoursPoly[i].size() < 50) & (contourArea(contours[i]) > 30)) {
            cv::drawContours(src, contoursPoly, i, color, 2, 8, hierarchy, 0, cv::Point());
            cv::minEnclosingCircle((cv::Mat)contoursPoly[i], center, radius);
            if (circlesCenters.empty()) {
                std::cout << "Circle: [Center - " << center << "]\t [Radius - " << radius << "]" << std::endl;
                circlesCenters.push_back(center);
                circlesRadii.push_back(radius);
            }
            else {
                valid = true;
                for (int j = 0; j < circlesCenters.size(); ++j) {
                    valid = valid && (abs(center.x - circlesCenters[j].x) > 2 || abs(center.y - circlesCenters[j].y) > 2);
                }
                if (valid) {
                    std::cout << "Circle: [Center - " << center << "]\t [Radius - " << radius << "]" << std::endl;
                    circlesCenters.push_back(center);
                    circlesRadii.push_back(radius);
                }
            }
        }
    }
}
