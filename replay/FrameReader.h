#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

class FrameReader
{
public:
    bool openVideo(const std::string& filename,
                   double captureFps);
    
    bool openPngSequence(const std::string& directory,
                     double captureFps);

    bool readFrame(int frameIndex, cv::Mat& frame);

    int totalFrames() const;
    double captureFps() const;
    double playbackFps() const;
    
    void close();

private:
    cv::VideoCapture cap_;

    int totalFrames_ = 0;
    double captureFps_ = 0.0;
    double playbackFps_ = 0.0;
    std::vector<std::string> pngFiles_;
};