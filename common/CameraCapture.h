
#pragma once

#include <opencv2/core.hpp>
#include <memory>

class CameraCapture {
public:
    CameraCapture();
    ~CameraCapture();

    // コピー禁止
    CameraCapture(const CameraCapture&) = delete;
    CameraCapture& operator=(const CameraCapture&) = delete;

    // カメラ初期化
    bool open(int width = 640, int height = 480,
              bool pgood_enable = true);

    // 撮影条件設定
    void setFps(int fps);
    void setExposure(int percent);
    void setGain(double gain_db);

    // 1フレーム取得（16bit画像）
    cv::Mat capture();

    // カメラ停止・解放
    void close();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
