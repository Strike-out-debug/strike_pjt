#include "FrameReader.h"
#include <filesystem>
#include <algorithm>
#include <opencv2/imgcodecs.hpp>

#include <iostream>

bool FrameReader::openPngSequence(const std::string& directory,
                                  double captureFps)
{
    close();

    if (captureFps <= 0.0) {
        std::cerr << "Invalid capture FPS" << std::endl;
        return false;
    }

    namespace fs = std::filesystem;

    try {
        for (const auto& entry : fs::directory_iterator(directory)) {
            if (!entry.is_regular_file()) {
                continue;
            }

            auto path = entry.path();

            if (path.extension() == ".png" ||
                path.extension() == ".PNG") {
                pngFiles_.push_back(path.string());
            }
        }
    }
    catch (const fs::filesystem_error& e) {
        std::cerr << "Directory error: " << e.what() << std::endl;
        close();
        return false;
    }

    std::sort(pngFiles_.begin(), pngFiles_.end());

    if (pngFiles_.empty()) {
        std::cerr << "No PNG files found" << std::endl;
        close();
        return false;
    }

    totalFrames_ = static_cast<int>(pngFiles_.size());
    captureFps_ = captureFps;
    playbackFps_ = 30.0;  // Replay display speed

    std::cout << "PNG sequence opened: " << directory << std::endl;
    std::cout << "Capture FPS: " << captureFps_ << std::endl;
    std::cout << "Total frames: " << totalFrames_ << std::endl;

    return true;
}

bool FrameReader::openVideo(const std::string& filename,
                            double captureFps)
{
    close();

    if (captureFps <= 0.0) {
        std::cerr << "Invalid capture FPS" << std::endl;
        return false;
    }

    if (!cap_.open(filename)) {
        std::cerr << "Failed to open video: "
                  << filename << std::endl;
        return false;
    }

    totalFrames_ = static_cast<int>(
        cap_.get(cv::CAP_PROP_FRAME_COUNT));

    playbackFps_ = cap_.get(cv::CAP_PROP_FPS);
    captureFps_ = captureFps;

    std::cout << "Video opened: " << filename << std::endl;
    std::cout << "Capture FPS: " << captureFps_ << std::endl;
    std::cout << "Playback FPS: " << playbackFps_ << std::endl;
    std::cout << "Total frames: " << totalFrames_ << std::endl;

    return true;
}

bool FrameReader::readFrame(int frameIndex, cv::Mat& frame)
{
    if (frameIndex < 0 ||
        (totalFrames_ > 0 && frameIndex >= totalFrames_)) {
        return false;
    }

    // PNG sequence
    if (!pngFiles_.empty()) {
        frame = cv::imread(
            pngFiles_[frameIndex],
            cv::IMREAD_UNCHANGED
        );

        return !frame.empty();
    }

    // MP4 / MOV video
    if (!cap_.isOpened()) {
        return false;
    }

    if (!cap_.set(cv::CAP_PROP_POS_FRAMES, frameIndex)) {
        return false;
    }

    return cap_.read(frame);
}

int FrameReader::totalFrames() const
{
    return totalFrames_;
}

double FrameReader::captureFps() const
{
    return captureFps_;
}

double FrameReader::playbackFps() const
{
    return playbackFps_;
}

void FrameReader::close()
{
    cap_.release();
    pngFiles_.clear();

    totalFrames_ = 0;
    captureFps_ = 0.0;
    playbackFps_ = 0.0;
}