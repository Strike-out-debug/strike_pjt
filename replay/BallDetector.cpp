#include "BallDetector.h"

#include <chrono>
#include <vector>

#include <opencv2/imgproc.hpp>

BallDetection BallDetector::detect(const cv::Mat& frame)
{
    const auto start = std::chrono::steady_clock::now();

    BallDetection result;

    if (frame.empty()) {
        return result;
    }

    // Convert input to 8-bit grayscale.
    cv::Mat gray;

    if (frame.type() == CV_16UC1) {
        // KV260: 10-bit RAW stored as 16-bit (x64).
        frame.convertTo(gray, CV_8U, 1.0 / 256.0);
    }
    else if (frame.type() == CV_8UC3) {
        // TG-7: BGR video.
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }
    else if (frame.type() == CV_8UC1) {
        gray = frame;
    }
    else {
        return result;
    }

    // First frame: store as reference.
    if (previousGray_.empty()) {
        previousGray_ = gray.clone();
        return result;
    }

    // Detect moving regions.
    cv::Mat diff;
    cv::subtract(previousGray_, gray, diff);

    cv::Mat binary;
    cv::threshold(diff, binary, 40, 255, cv::THRESH_BINARY);

    // Remove isolated noise.
    cv::morphologyEx(
        binary,
        binary,
        cv::MORPH_OPEN,
        cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(3, 3)
        )
    );
    debugBinary_ = binary.clone();

    // Extract candidate contours.
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(
        binary,
        contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE
    );

    // Initial settings for detection testing.
    constexpr double minArea = 30.0;
    constexpr double maxArea = 10000.0;

    double bestArea = 0.0;

    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);

        if (area < minArea || area > maxArea) {
            continue;
        }

        cv::Point2f center;
        float radius;

        cv::minEnclosingCircle(contour, center, radius);

        // Reject candidates touching the image boundary.
        if (center.x - radius <= 0 ||
            center.y - radius <= 0 ||
            center.x + radius >= frame.cols - 1 ||
            center.y + radius >= frame.rows - 1) {
            continue;
        }

        if (area > bestArea) {
            bestArea = area;
            result.detected = true;
            result.center = center;
            result.diameter = radius * 2.0f;
        }
    }

    // Update previous frame.
    previousGray_ = gray.clone();

    const auto end = std::chrono::steady_clock::now();

    result.processingUs =
        std::chrono::duration<double, std::micro>(
            end - start
        ).count();

    return result;
}

void BallDetector::reset()
{
    previousGray_.release();
    debugBinary_.release();
}

cv::Mat BallDetector::debugBinary() const
{
    return debugBinary_.clone();
}
