#include <unistd.h>
#include <utility>
#include <algorithm>
#include <iostream>

#include "CameraCapture.h"

#include "jelly/UioAccessor.h"
#include "jelly/UdmabufAccessor.h"
#include "jelly/JellyRegs.h"
#include "jelly/I2cAccessor.h"
#include "jelly/GpioAccessor.h"
#include "jelly/VideoDmaControl.h"
#include "rtcl/RtclP3S7Control.h"

// FPGA system registers (from original main.cpp)
#define SYSREG_ID                   0x0000
#define SYSREG_DPHY_SW_RESET        0x0001
#define SYSREG_CAM_ENABLE           0x0002
#define SYSREG_DPHY_INIT_DONE       0x0004
#define SYSREG_IMAGE_WIDTH          0x0008
#define SYSREG_IMAGE_HEIGHT         0x0009
#define SYSREG_BLACK_WIDTH          0x000a
#define SYSREG_BLACK_HEIGHT         0x000b

// FPGA timing generator registers
#define TIMGENREG_CTL_CONTROL       0x04
#define TIMGENREG_PARAM_PERIOD      0x10
#define TIMGENREG_PARAM_TRIG0_START 0x20
#define TIMGENREG_PARAM_TRIG0_END   0x21

struct CameraCapture::Impl {
    int width = 640;
    int height = 480;
    int fps = 200;
    int exposure = 90;
    double gain_db = 10.0;
    bool opened = false;
    bool camera_powered = false;
    bool sensor_enabled = false;

    bool module_enabled = false;
    bool video_started = false;
    bool sequencer_enabled = false;

    // ハードウェア制御オブジェクト
    std::unique_ptr<jelly::UioAccessor> uio;
    std::unique_ptr<jelly::UdmabufAccessor> dmabuf0;
    std::unique_ptr<jelly::UdmabufAccessor> dmabuf1;
    std::unique_ptr<rtcl::RtclP3S7ControlI2c> cam;
    std::unique_ptr<jelly::VideoDmaControl> vdma0;

    
    // FPGAレジスタアクセサの型
    using RegAccessor = decltype(
        std::declval<jelly::UioAccessor&>().GetAccessor(0));

    RegAccessor reg_sys;
    RegAccessor reg_timgen;
    RegAccessor reg_fmtr;
    RegAccessor reg_wdma0;
    RegAccessor reg_wdma1;

};

CameraCapture::CameraCapture()
    : impl_(std::make_unique<Impl>())
{
}

CameraCapture::~CameraCapture()
{
    close();
}


void CameraCapture::setFps(int fps)
{
    if (fps < 5 || fps > 1000) {
        return;
    }

    impl_->fps = fps;

    if (!impl_->opened) {
        return;
    }

    int period = 100000000 / impl_->fps;
    int trig_end = period * impl_->exposure / 100;

    impl_->reg_timgen.WriteReg(
        TIMGENREG_PARAM_PERIOD, period - 1);
    impl_->reg_timgen.WriteReg(
        TIMGENREG_PARAM_TRIG0_START, 1);
    impl_->reg_timgen.WriteReg(
        TIMGENREG_PARAM_TRIG0_END, trig_end);
    impl_->reg_timgen.WriteReg(
        TIMGENREG_CTL_CONTROL, 3);
}

void CameraCapture::setExposure(int percent)
{
    if (percent < 10 || percent > 90) {
        return;
    }

    impl_->exposure = percent;

    if (impl_->opened) {
        setFps(impl_->fps);
    }
}

void CameraCapture::setGain(double gain_db)
{
    if (gain_db < 0.0 || gain_db > 10.0) {
        return;
    }

    impl_->gain_db = gain_db;

    if (impl_->opened && impl_->cam) {
        impl_->cam->SetGainDb(
            static_cast<float>(gain_db));
    }
}


void CameraCapture::close()
{
    if (!impl_) {
        return;
    }
    
    // Stop FPGA trigger generator
    if (impl_->uio && impl_->module_enabled) {
        impl_->reg_timgen.WriteReg(
            TIMGENREG_CTL_CONTROL, 0);
        usleep(10000);
    }

    // 1. Stop video input
    if (impl_->video_started) {
        impl_->reg_fmtr.WriteReg(
            REG_VIDEO_FMTREG_CTL_CONTROL, 0x0);
        usleep(100000);
        impl_->video_started = false;
    }

    // 2. Stop sequencer
    if (impl_->cam && impl_->sequencer_enabled) {
        impl_->cam->SetSequencerEnable(false);
        usleep(10000);
        impl_->sequencer_enabled = false;
    }

    // 3. Stop sensor
    if (impl_->cam && impl_->sensor_enabled) {
        impl_->cam->SetSensorEnable(false);
        usleep(10000);
        impl_->sensor_enabled = false;
    }

    // 4. Turn off sensor power
    if (impl_->cam && impl_->camera_powered) {
        impl_->cam->SetSensorPowerEnable(false);
        usleep(10000);
        impl_->camera_powered = false;
    }

    // 5. Turn off camera module
    if (impl_->uio && impl_->module_enabled) {
        impl_->reg_sys.WriteReg(SYSREG_CAM_ENABLE, 0);
        usleep(100000);
        impl_->module_enabled = false;
    }

    // 6. Release DMA and camera resources
    impl_->vdma0.reset();
    impl_->cam.reset();
    impl_->dmabuf1.reset();
    impl_->dmabuf0.reset();

    // Release register accessors before UIO mapping
    impl_->reg_sys = {};
    impl_->reg_timgen = {};
    impl_->reg_fmtr = {};
    impl_->reg_wdma0 = {};
    impl_->reg_wdma1 = {};

    impl_->uio.reset();
    impl_->opened = false;
}


bool CameraCapture::open(int width, int height, bool pgood_enable)
{
    if (impl_->opened) {
    return false;
    }

    // Clean up resources left by a previous incomplete open().
    close();

    // オリジナルと同じ画像サイズ補正
    impl_->width = std::max(width & ~0xf, 16);
    impl_->height = std::max(height, 1);

    // FPGA UIOアクセス
    impl_->uio = std::make_unique<jelly::UioAccessor>(
        "uio_pl_peri", 0x08000000);

    if (!impl_->uio->IsMapped()) {
        std::cerr << "uio_pl_peri mmap error" << std::endl;
        impl_->uio.reset();
        return false;
    }

    // FPGAレジスタアクセサ取得
    impl_->reg_sys =
        impl_->uio->GetAccessor(0x00000000);
    impl_->reg_timgen =
        impl_->uio->GetAccessor(0x00010000);
    impl_->reg_fmtr =
        impl_->uio->GetAccessor(0x00100000);
    impl_->reg_wdma0 =
        impl_->uio->GetAccessor(0x00210000);
    impl_->reg_wdma1 =
        impl_->uio->GetAccessor(0x00220000);

    // DMAバッファ0
    impl_->dmabuf0 =
        std::make_unique<jelly::UdmabufAccessor>(
            "udmabuf-jelly-vram0");

    if (!impl_->dmabuf0->IsMapped()) {
        std::cerr << "udmabuf0 mmap error" << std::endl;
        close();
        return false;
    }

    std::cout << "udmabuf0 size : "
              << impl_->dmabuf0->GetSize() << std::endl;

    
    // 1フレームに必要なDMAバッファ容量
    const std::size_t frame_bytes =
        static_cast<std::size_t>(impl_->width) *
        static_cast<std::size_t>(impl_->height) * 2;

    if (frame_bytes > impl_->dmabuf0->GetSize()) {
        std::cerr << "DMA buffer too small: "
                  << frame_bytes << " bytes required"
                  << std::endl;

        close();
        return false;
    }

    // DMAバッファ1
    impl_->dmabuf1 =
        std::make_unique<jelly::UdmabufAccessor>(
            "udmabuf-jelly-vram1");

    if (!impl_->dmabuf1->IsMapped()) {
        std::cerr << "udmabuf1 mmap error" << std::endl;
        close();
        return false;
    }

    std::cout << "udmabuf1 size : "
              << impl_->dmabuf1->GetSize() << std::endl;

    
    // I2Cカメラ制御オブジェクト生成
    impl_->cam =
        std::make_unique<rtcl::RtclP3S7ControlI2c>();

    // カメラ制御用I2C接続
    impl_->cam->Open("/dev/i2c-6", 0x10);
    
    // Reset camera module (same sequence as original)
    impl_->reg_sys.WriteReg(SYSREG_CAM_ENABLE, 0);
    usleep(10000);

    impl_->reg_sys.WriteReg(SYSREG_CAM_ENABLE, 1);
    impl_->module_enabled = true;
    usleep(10000);

    
    // Camera module information
    std::cout << "Camera Module ID      : "
              << std::hex << impl_->cam->GetModuleId()
              << std::endl;

    std::cout << "Camera Module Version : "
              << std::hex << impl_->cam->GetModuleVersion()
              << std::dec << std::endl;

    // Set DPHY speed: 1250 Mbps
    impl_->cam->SetDphySpeed(1250000000);

    // Sensor power-good monitoring
    std::cout << "Sensor PGood Enable : "
              << (pgood_enable ? "ON" : "OFF")
              << std::endl;

    impl_->cam->SetSensorPGoodEnable(pgood_enable);

    // Reset receiver DPHY
    impl_->reg_sys.WriteReg(SYSREG_DPHY_SW_RESET, 1);

    // Initialize camera board
    std::cout << "Init Camera" << std::endl;

    impl_->cam->SetSensorPowerEnable(false);
    impl_->cam->SetDphyReset(true);
    usleep(10000);

    // Release receiver DPHY reset FIRST
    impl_->reg_sys.WriteReg(SYSREG_DPHY_SW_RESET, 0);
    usleep(10000);

    // Set high-speed camera mode
    impl_->cam->SetCameraMode(
        rtcl::RtclP3S7ControlI2c::MODE_HIGH_SPEED);

    // Turn on sensor power
    std::cout << "Sensor Power On" << std::endl;
    impl_->cam->SetSensorPowerEnable(true);
    impl_->camera_powered = true;
    usleep(10000);

    // Release camera-side DPHY TX reset
    impl_->cam->SetDphyReset(false);

    // Check camera-side DPHY initialization
    if (!impl_->cam->GetDphyInitDone()) {
        std::cerr
            << "!!ERROR!! CAM DPHY TX init_done = 0"
            << std::endl;
        close();
        return false;
    }

    // Check KV260-side DPHY RX initialization
    auto dphy_rx_init_done =
        impl_->reg_sys.ReadReg(SYSREG_DPHY_INIT_DONE);

    if (dphy_rx_init_done == 0) {
        std::cerr
            << "!!ERROR!! KV260 DPHY RX init_done = 0"
            << std::endl;
        close();
        return false;
    }

    // Read sensor ID
    std::cout << "Sensor ID : "
              << impl_->cam->GetSensorId()
              << std::endl;

    
    // Configure image size
    impl_->reg_sys.WriteReg(
        SYSREG_IMAGE_WIDTH, impl_->width);
    impl_->reg_sys.WriteReg(
        SYSREG_IMAGE_HEIGHT, impl_->height);
    impl_->reg_sys.WriteReg(
        SYSREG_BLACK_WIDTH, 1280);
    impl_->reg_sys.WriteReg(
        SYSREG_BLACK_HEIGHT, 1);

    // Configure sensor timing
    auto xsm_delay =
        impl_->cam->CalcXsmDelay(impl_->width);

    impl_->cam->SetXsmDelay(xsm_delay);
    impl_->cam->SetNzrotXsmDelayEnable(true);
    impl_->cam->SetZeroRotEnable(true);

    
    // Enable sensor
    bool sensor_ok = impl_->cam->SetSensorEnable(true);

    // Ensure cleanup attempts to disable the sensor
    impl_->sensor_enabled = true;

    if (!sensor_ok) {
        std::cerr << "Sensor enable failed"
                  << std::endl;

        if (!impl_->cam->GetSensorPGood()) {
            std::cerr
                << "Sensor power-good error"
                << std::endl;
        }

        close();
        return false;
    }

    // Configure sensor ROI
    impl_->cam->SetRoi0(
        impl_->width, impl_->height);

    
    // Configure FPGA video formatter
    impl_->reg_fmtr.WriteReg(
        REG_VIDEO_FMTREG_CTL_FRM_TIMER_EN, 1);

    impl_->reg_fmtr.WriteReg(
        REG_VIDEO_FMTREG_CTL_FRM_TIMEOUT, 20000000);

    impl_->reg_fmtr.WriteReg(
        REG_VIDEO_FMTREG_PARAM_WIDTH, impl_->width);

    impl_->reg_fmtr.WriteReg(
        REG_VIDEO_FMTREG_PARAM_HEIGHT, impl_->height);

    impl_->reg_fmtr.WriteReg(
        REG_VIDEO_FMTREG_PARAM_FILL, 0xffff);

    impl_->reg_fmtr.WriteReg(
        REG_VIDEO_FMTREG_PARAM_TIMEOUT, 100000);

    // Start video formatter
    impl_->reg_fmtr.WriteReg(
        REG_VIDEO_FMTREG_CTL_CONTROL, 0x03);

    impl_->video_started = true;

    
    // Configure sensor timing and trigger mode
    impl_->cam->SetMultTimer0(72);
    impl_->cam->SetFrLength0(0);
    impl_->cam->SetExposure0(10000);

    impl_->cam->SetTriggeredMode(true);
    impl_->cam->SetSlaveMode(true);

    // Start sensor sequencer
    impl_->cam->SetSequencerEnable(true);
    impl_->sequencer_enabled = true;
    usleep(100000);

    // Set initial gain
    impl_->cam->SetGainDb(10.0);

    
    // Initialize video DMA controller
    impl_->vdma0 = std::make_unique<jelly::VideoDmaControl>(
        impl_->reg_wdma0, 2, 2, true);

    std::cout << "Video DMA initialized"
              << std::endl;

    
    // Camera initialization completed
    impl_->opened = true;

    // Apply initial FPS and exposure settings
    setFps(impl_->fps);

    // Apply initial gain setting
    setGain(impl_->gain_db);

    std::cout << "Camera initialized successfully"
              << std::endl;

    return true;

}


cv::Mat CameraCapture::capture()
{
    // Return empty image if camera is not ready
    if (!impl_->opened ||
        !impl_->vdma0 ||
        !impl_->dmabuf0) {
        return cv::Mat();
    }

    // Get DMA buffer physical address
    auto phys_addr = impl_->dmabuf0->GetPhysAddr();

    // Capture one frame
    impl_->vdma0->Oneshot(
        phys_addr,
        impl_->width,
        impl_->height,
        1);

    // Copy raw image from DMA buffer
    cv::Mat img(
        impl_->height,
        impl_->width,
        CV_16U);

    impl_->dmabuf0->MemCopyTo(
        img.data,
        0,
        impl_->width * impl_->height * 2);

    // Convert 10-bit image to 16-bit range
    return img * 64;
}

