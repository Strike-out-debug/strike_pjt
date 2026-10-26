#include <iostream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char* argv[])
{
    constexpr double captureFps = 480.0;

    if (argc < 2) {
        std::cerr << "Usage: strike_replay <video_file>\n";
        return 1;
    }

    cv::VideoCapture cap(argv[1]);

    if (!cap.isOpened()) {
        std::cerr << "Failed to open video\n";
        return 1;
    }

    const int totalFrames =
        static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));

    const double playbackFps = cap.get(cv::CAP_PROP_FPS);

    std::cout << "STRIKE Replay\n";
    std::cout << "Capture FPS: " << captureFps << "\n";
    std::cout << "Playback FPS: " << playbackFps << "\n";
    std::cout << "Total frames: " << totalFrames << "\n";
    std::cout << "SPACE: Play/Pause\n";
    std::cout << "N: Next frame\n";
    std::cout << "P: Previous frame\n";
    std::cout << "J: Jump to frame\n";
    std::cout << "ESC: Exit\n";

    cv::namedWindow("STRIKE Replay", cv::WINDOW_AUTOSIZE);

    bool paused = true;
    int frameIndex = 0;

    while (true) {
        cap.set(cv::CAP_PROP_POS_FRAMES, frameIndex);

        cv::Mat frame;
        if (!cap.read(frame)) {
            std::cout << "End of video\n";
            break;
        }

        double timeMs = frameIndex * 1000.0 / captureFps;

        std::string info =
            "Frame: " + std::to_string(frameIndex) +
            " / " + std::to_string(totalFrames - 1) +
            "  Time: " + std::to_string(timeMs) + " ms";

        // Black outline
        cv::putText(
            frame, info, cv::Point(10, 25),
            cv::FONT_HERSHEY_SIMPLEX, 0.45,
            cv::Scalar(0, 0, 0), 3
        );

        // White text
        cv::putText(
            frame, info, cv::Point(10, 25),
            cv::FONT_HERSHEY_SIMPLEX, 0.45,
            cv::Scalar(255, 255, 255), 1
        );
        
        cv::imshow("STRIKE Replay", frame);

        int delay = paused ? 0 :
            (playbackFps > 0
                ? static_cast<int>(1000.0 / playbackFps)
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
            if (!(std::cin >> target)) {
                std::cin.clear();
                std::string ignored;
                std::getline(std::cin, ignored);
                continue;
            }

            frameIndex = target;
        }
        else if (!paused) {
            frameIndex++;
        }

        if (totalFrames > 0) {
            frameIndex = cv::max(0, cv::min(frameIndex, totalFrames - 1));
        } else {
            frameIndex = cv::max(0, frameIndex);
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}