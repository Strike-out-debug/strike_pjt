#include <iostream>
#include <string>
#include <algorithm>
#include <filesystem>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "FrameReader.h"

cv::Mat makeDisplayImage(const cv::Mat& raw)
{
    // TG-7 video: 8-bit BGR
    if (raw.type() == CV_8UC3) {
        return raw.clone();
    }

    // KV260: 16-bit RAW Bayer
    if (raw.type() == CV_16UC1) {
        cv::Mat raw8;
        cv::Mat bgr;

        // KV260 stores 10-bit data scaled by 64
        raw.convertTo(raw8, CV_8U, 1.0 / 256.0);

        // Bayer BG -> BGR
        cv::cvtColor(
            raw8,
            bgr,
            cv::COLOR_BayerBG2BGR
        );

        return bgr;
    }

    return cv::Mat();
}

int main(int argc, char* argv[])
{
    const double captureFps = (argc >= 3) ? std::stod(argv[2]) : 480.0;

    if (argc < 2) {
        std::cerr << "Usage: strike_replay <video_file>\n";
        return 1;
    }

    FrameReader reader;

    namespace fs = std::filesystem;

    bool opened = false;

    if (fs::is_directory(argv[1])) {
        // KV260 PNG sequence
        opened = reader.openPngSequence(argv[1], captureFps);
    }
        else {
        // TG-7 MP4 / MOV
        opened = reader.openVideo(argv[1], captureFps);
    }   

    if (!opened) {
        return 1;
    }

    const int totalFrames = reader.totalFrames();
    const double playbackFps = reader.playbackFps();

    std::cout << "STRIKE Replay\n";
    std::cout << "SPACE: Play/Pause\n";
    std::cout << "N: Next frame\n";
    std::cout << "P: Previous frame\n";
    std::cout << "J: Jump to frame\n";
    std::cout << "ESC: Exit\n";

    cv::namedWindow("STRIKE Replay", cv::WINDOW_AUTOSIZE);

    bool paused = true;
    int frameIndex = 0;

    while (true) {
        cv::Mat frame;

        if (!reader.readFrame(frameIndex, frame)) {
            std::cerr << "Failed to read frame: "
                      << frameIndex << "\n";
            break;
        }

        cv::Mat displayFrame = makeDisplayImage(frame);

        if (displayFrame.empty()) {
            std::cerr << "Unsupported image format\n";
            break;
        }

        double timeMs =
            frameIndex * 1000.0 / reader.captureFps();

        std::string info =
            "Frame: " + std::to_string(frameIndex) +
            " / " + std::to_string(totalFrames - 1) +
            "  Time: " + std::to_string(timeMs) + " ms";

        // Black outline
        cv::putText(
            displayFrame, info, cv::Point(10, 25),
            cv::FONT_HERSHEY_SIMPLEX, 0.45,
            cv::Scalar(0, 0, 0), 3
        );

        // White text
        cv::putText(
            displayFrame, info, cv::Point(10, 25),
            cv::FONT_HERSHEY_SIMPLEX, 0.45,
            cv::Scalar(255, 255, 255), 1
        );

        cv::imshow("STRIKE Replay", displayFrame);

        int delay = paused ? 0 :
            (playbackFps > 0
                ? std::max(1, static_cast<int>(1000.0 / playbackFps))
                : 33);

        int key = cv::waitKey(delay);

        if (key == 27) {
            break;
        }
        else if (key == ' ') {
            paused = !paused;
        }
        else if (key == 'n' || key == 'N') {
            paused = true;
            frameIndex++;
        }
        else if (key == 'p' || key == 'P') {
            paused = true;
            frameIndex--;
        }
        else if (key == 'j' || key == 'J') {
            paused = true;

            std::cout << "Jump to frame: ";
            int target;

            if (std::cin >> target) {
                frameIndex = target;
            } else {
                std::cin.clear();
                std::string ignored;
                std::getline(std::cin, ignored);
            }
        }
        else if (!paused) {
            frameIndex++;
        }

        if (totalFrames > 0) {
            frameIndex = std::clamp(frameIndex, 0, totalFrames - 1);
        } else {
            frameIndex = std::max(0, frameIndex);
        }
    }

    reader.close();
    cv::destroyAllWindows();

    return 0;
}