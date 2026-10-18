
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include "CameraCapture.h"

int main()
{
    std::cout << "CameraCapture test start"
              << std::endl;

    CameraCapture camera;

    camera.setFps(200);
    camera.setExposure(90);
    camera.setGain(10.0);


    std::cout << "Opening camera..." << std::endl;

    if (!camera.open(640, 480, true)) {
        std::cerr << "Camera open failed" << std::endl;
        return 1;
    }


    std::cout << "Camera open succeeded" << std::endl;

    // Capture one frame
    cv::Mat img = camera.capture();

    if (img.empty()) {
        std::cerr << "Capture failed" << std::endl;
        camera.close();
        return 1;
    }

    std::cout << "Image size: "
              << img.cols << " x " << img.rows
              << std::endl;

    // Save 16-bit raw Bayer image as PNG
    if (!cv::imwrite("camera_test.png", img)) {
        std::cerr << "Image save failed" << std::endl;
        camera.close();
        return 1;
    }

    std::cout << "Saved camera_test.png" << std::endl;

    camera.close();

    std::cout << "Camera closed successfully"
              << std::endl;

    std::cout << "CameraCapture object created"
              << std::endl;

    return 0;
}
