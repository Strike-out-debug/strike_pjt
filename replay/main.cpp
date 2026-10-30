#include <fstream>
#include <iomanip>
#include "BallDetector.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <filesystem>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
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

struct PlateEdge
{
    cv::Point2f points[2];
    int count = 0;
};

void onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event != cv::EVENT_RBUTTONDOWN) {
        return;
    }

    auto* edge = static_cast<PlateEdge*>(userdata);

    if (edge == nullptr) {
        return;
    }

    if (edge->count >= 2) {
        edge->count = 0;
    }

    edge->points[edge->count] =
        cv::Point2f(static_cast<float>(x),
                    static_cast<float>(y));

    edge->count++;

    if (edge->count == 2) {
        double length = cv::norm(
            edge->points[1] - edge->points[0]
        );

        double targetDiameter =
            length * 73.0 / 431.8;

        std::cout << "Plate edge: "
                  << length << " px\n";

        std::cout << "Target ball diameter: "
                  << targetDiameter << " px\n";
    }
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
    
    // Analyze all frames and export detection results to CSV.
    std::filesystem::create_directories("out");

    std::ofstream csv("out/ball_detection.csv");

    if (!csv) {
        std::cerr << "Failed to create CSV file\n";
        return 1;
    }

    csv << "frame,time_ms,detected,center_x,center_y,"
        "diameter_px,processing_us\n";

    csv << std::fixed << std::setprecision(3);

    BallDetector csvDetector;

    for (int i = 0; i < totalFrames; ++i) {
        cv::Mat rawFrame;

        if (!reader.readFrame(i, rawFrame)) {
            std::cerr << "Failed to analyze frame: " << i << "\n";
            break;
       }

        BallDetection result = csvDetector.detect(rawFrame);

        // Save debug images for frames 1030-1034.
        if (i >= 1030 && i <= 1034) {
            std::filesystem::create_directories("out/debug");

            // Original frame
            cv::imwrite(
            "out/debug/frame_" + std::to_string(i) + ".png",
            rawFrame
            );

            // Binary difference image
            cv::Mat binary = csvDetector.debugBinary();

            if (!binary.empty()) {
                cv::imwrite(
                    "out/debug/diff_" + std::to_string(i) + ".png",
                    binary
                );
            }
        }

        double timeMs = i * 1000.0 / reader.captureFps();

        csv << i << ","
            << timeMs << ","
            << (result.detected ? 1 : 0) << ",";

        if (result.detected) {
            csv << result.center.x << ","
            << result.center.y << ","
            << result.diameter;
        } else {
            csv << ",,";
        }

        csv << "," << result.processingUs << "\n";
    }

    csv.close();
    csvDetector.reset();

    std::cout << "CSV saved: out/ball_detection.csv\n";

    std::cout << "STRIKE Replay\n";
    std::cout << "SPACE: Play/Pause\n";
    std::cout << "N: Next frame\n";
    std::cout << "P: Previous frame\n";
    std::cout << "J: Jump to frame\n";
    std::cout << "ESC: Exit\n";

    cv::namedWindow("STRIKE Replay", cv::WINDOW_AUTOSIZE);
    PlateEdge plateEdge;
    cv::setMouseCallback("STRIKE Replay", onMouse, &plateEdge);

    bool paused = true;
    int frameIndex = 0;
    BallDetector detector;
    int previousFrameIndex = -1;

    while (true) {
        cv::Mat frame;

        if (!reader.readFrame(frameIndex, frame)) {
            std::cerr << "Failed to read frame: "
                      << frameIndex << "\n";
            break;
        }

        
        // Reset differential detection after a jump or backward seek.
        if (frameIndex != previousFrameIndex + 1) {
            detector.reset();
        }

        // Detect using the original image.
        BallDetection ball = detector.detect(frame);
        previousFrameIndex = frameIndex;

        cv::Mat displayFrame = makeDisplayImage(frame);

        if (displayFrame.empty()) {
            std::cerr << "Unsupported image format\n";
            break;
        }

        // Draw detected ball candidate.
        if (ball.detected) {
            cv::circle(
            displayFrame,
            ball.center,
            static_cast<int>(ball.diameter / 2.0f),
            cv::Scalar(0, 0, 255),
            2
            );

            cv::circle(
                displayFrame,
                ball.center,
                3,
                cv::Scalar(0, 255, 255),
                -1
            );
        }   

        // Display detection result and processing time.
        std::string ballInfo;

        if (ball.detected) {
            ballInfo =
            "Ball: (" +
            std::to_string(static_cast<int>(ball.center.x)) + "," +
            std::to_string(static_cast<int>(ball.center.y)) + ")" +
            " D=" + std::to_string(static_cast<int>(ball.diameter)) + "px";
        }
        else {
            ballInfo = "Ball: NOT DETECTED";
        }

        ballInfo += "  " +
            std::to_string(static_cast<int>(ball.processingUs)) + "us";

        // Black outline
        cv::putText(
            displayFrame, ballInfo, cv::Point(10, 45),
            cv::FONT_HERSHEY_SIMPLEX, 0.40,
            cv::Scalar(0, 0, 0), 3
        );

        // White text
        cv::putText(
            displayFrame, ballInfo, cv::Point(10, 45),
            cv::FONT_HERSHEY_SIMPLEX, 0.40,
            cv::Scalar(255, 255, 255), 1
        );

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