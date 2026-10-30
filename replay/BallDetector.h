#pragma once

#include <opencv2/core.hpp>

struct BallDetection
{
    bool detected = false;

    cv::Point2f center = {0.0f, 0.0f};
    float diameter = 0.0f;

    double processingUs = 0.0;
};

class BallDetector
{
public:
    BallDetection detect(const cv::Mat& frame);

    void reset();

    // Get the latest binary difference image.
    cv::Mat debugBinary() const;

private:
    cv::Mat previousGray_;
    cv::Mat debugBinary_;
};
