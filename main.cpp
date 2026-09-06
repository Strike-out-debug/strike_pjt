#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <deque>
#include <thread>
#include <mutex>
#include <termios.h>
#include <opencv2/opencv.hpp>
#include "jelly/UioAccessor.h"
#include "jelly/UdmabufAccessor.h"
#include "jelly/JellyRegs.h"
#include "jelly/I2cAccessor.h"
#include "jelly/GpioAccessor.h"
#include "jelly/VideoDmaControl.h"
#include "rtcl/RtclP3S7Control.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <atomic>
#include <cmath>
std::atomic<bool> save_request(false);

// BLE受信時に直前1秒分だけ保存
std::deque<cv::Mat> pre_buffer;
#define SYSREG_ID                   0x0000
#define SYSREG_DPHY_SW_RESET        0x0001
#define SYSREG_CAM_ENABLE           0x0002
#define SYSREG_CSI_DATA_TYPE        0x0003
#define SYSREG_DPHY_INIT_DONE       0x0004
#define SYSREG_FPS_COUNT            0x0006
#define SYSREG_FRAME_COUNT          0x0007
#define SYSREG_IMAGE_WIDTH          0x0008
#define SYSREG_IMAGE_HEIGHT         0x0009
#define SYSREG_BLACK_WIDTH          0x000a
#define SYSREG_BLACK_HEIGHT         0x000b
#define TIMGENREG_CORE_ID           0x00
#define TIMGENREG_CORE_VERSION      0x01
#define TIMGENREG_CTL_CONTROL       0x04
#define TIMGENREG_CTL_STATUS        0x05
#define TIMGENREG_CTL_TIMER         0x08
#define TIMGENREG_PARAM_PERIOD      0x10
#define TIMGENREG_PARAM_TRIG0_START 0x20
#define TIMGENREG_PARAM_TRIG0_END   0x21
#define TIMGENREG_PARAM_TRIG0_POL   0x22
void          sensor_reg_dump(rtcl::RtclP3S7ControlI2c  &cam, const char *fname);
void          load_setting(rtcl::RtclP3S7ControlI2c  &cam);

static  volatile    bool    g_signal = false;
void signal_handler(int signo) {
    g_signal = true;
}

// ------------------------------------------------------------
// ホームベース前縁の2点計測
// 左クリック: 1点目 → 2点目
// 右クリック: リセット
// ------------------------------------------------------------
static cv::Point base_point1(-1, -1);
static cv::Point base_point2(-1, -1);
static int base_point_count = 0;
static double base_edge_pixels = 0.0;
static double target_ball_pixels = 0.0;

// 硬式野球ボールの代表値
static constexpr double BASEBALL_DIAMETER_MM = 73.0;

// ホームベースの前縁は17インチ = 431.8 mm
static constexpr double HOME_PLATE_FRONT_EDGE_MM = 431.8;

void on_mouse(int event, int x, int y, int flags, void* userdata)
{
    (void)flags;
    (void)userdata;

    if (event == cv::EVENT_LBUTTONDOWN)
    {
        if (base_point_count == 0)
        {
            base_point1 = cv::Point(x, y);
            base_point_count = 1;
            base_edge_pixels = 0.0;
            target_ball_pixels = 0.0;

            std::cout << "Base point 1 = ("
                      << x << ", " << y << ")" << std::endl;
        }
        else
        {
            base_point2 = cv::Point(x, y);
            base_point_count = 2;

            double dx = static_cast<double>(base_point2.x - base_point1.x);
            double dy = static_cast<double>(base_point2.y - base_point1.y);

            base_edge_pixels = std::sqrt(dx * dx + dy * dy);

            target_ball_pixels =
                base_edge_pixels *
                BASEBALL_DIAMETER_MM /
                HOME_PLATE_FRONT_EDGE_MM;

            std::cout << "Base point 2 = ("
                      << x << ", " << y << ")" << std::endl;

            std::cout << "Home plate front edge = "
                      << base_edge_pixels << " pixel" << std::endl;

            std::cout << "Target baseball diameter = "
                      << target_ball_pixels << " pixel"
                      << std::endl;

            std::cout << "Right click to reset." << std::endl;
        }
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        base_point1 = cv::Point(-1, -1);
        base_point2 = cv::Point(-1, -1);
        base_point_count = 0;
        base_edge_pixels = 0.0;
        target_ball_pixels = 0.0;

        std::cout << "Home plate measurement reset." << std::endl;
    }
}

void uart_thread()
{
    int fd = open("/dev/ttyUSB0", O_RDONLY | O_NOCTTY);

    if (fd < 0)
    {
        perror("open");
        return;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0)
    {
        perror("tcgetattr");
        close(fd);
        return;
    }

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CLOCAL | CREAD;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag = 0;
    tty.c_iflag &= ~(ICRNL | INLCR | IGNCR);
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 0;

    tcflush(fd, TCIFLUSH);

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        perror("tcsetattr");
        close(fd);
        return;
    }

    std::cout << "UART thread started" << std::endl;

    while (true)
    {
        unsigned char buf[64];

        int ret = read(fd, buf, sizeof(buf));

        if (ret > 0)
        {
            printf("RX %d byte(s): ", ret);
            for (int i = 0; i < ret; i++)
            {
                printf("%02X ", buf[i]);

                if (buf[i] == 0x01)
                {
                    std::cout << "SAVE trigger received" << std::endl;
                    save_request = true;
                }
            }

            printf("\n");
        }

        usleep(1000);
    }
}
// ------------------------------------------------------------
// PNG保存用ヘルパー
// img_raw は CV_16U の10bit画像(0～1023)として扱う。
// 8bit化してから保存することで、通常のPNGビューアでも
// 正しい明るさで確認できるようにする。
// color=true の場合は BayerBG -> BGR に変換して保存する。
// ------------------------------------------------------------
static bool save_frame_png(const cv::Mat& raw, const char* fname, bool color)
{
    if (raw.empty() || raw.type() != CV_16U)
    {
        std::cerr << "save_frame_png: invalid image" << std::endl;
        return false;
    }

    double min_val = 0.0;
    double max_val = 0.0;
    cv::minMaxLoc(raw, &min_val, &max_val);

    // 10bit (0～1023) -> 8bit (0～255)
    cv::Mat img8;
    raw.convertTo(img8, CV_8U, 255.0 / 1023.0);

    cv::Mat save_img;

    if (color)
    {
        cv::cvtColor(img8, save_img, cv::COLOR_BayerBG2BGR);
    }
    else
    {
        save_img = img8;
    }

    bool ok = cv::imwrite(fname, save_img);

    if (!ok)
    {
        std::cerr << "imwrite failed: " << fname << std::endl;
    }

    std::cout << "Saved " << fname
              << " raw_min=" << min_val
              << " raw_max=" << max_val
              << std::endl;

    return ok;
}

// メイン関数
int main(int argc, char *argv[])
{
    int width  = 640 ;
    int height = 480 ;
    int fps    = 200 ;
    int exposure = 90;  // 90%
    int gain     = 95;  // 0.0 db
    bool color = true;
    bool pgood_enable = true;
    for ( int i = 1; i < argc; ++i ) {
        if ( (strcmp(argv[i], "-W") == 0 || strcmp(argv[i], "--width") == 0) && i+1 < argc) {
            ++i;
            width = strtol(argv[i], nullptr, 0);
        }
        else if ( (strcmp(argv[i], "-H") == 0 || strcmp(argv[i], "--height") == 0) && i+1 < argc) {
            ++i;
            height = strtol(argv[i], nullptr, 0);
        }
        else if ( (strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--fps") == 0) && i+1 < argc) {
            ++i;
            fps = strtol(argv[i], nullptr, 0);
        }
        else if ( strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--color") == 0 ) {
            color = true;
        }
        else if ( strcmp(argv[i], "--pgood-off") == 0 ) {
            pgood_enable = false;
        }
        else {
            std::cout << "unknown option : " << argv[i] << std::endl;
            return 1;
        }
    }
    std::cout << "width  : " << width << std::endl;
    std::cout << "height : " << height << std::endl;
    std::cout << "color  : " << color << std::endl;

    width &= ~0xf;
    width  = std::max(width, 16);
    height = std::max(height, 1);

    signal(SIGINT, signal_handler);

    jelly::UioAccessor uio_acc("uio_pl_peri", 0x08000000);
    if ( !uio_acc.IsMapped() ) {
        std::cout << "uio_pl_peri mmap error" << std::endl;
        return 1;
    }
    auto reg_sys    = uio_acc.GetAccessor(0x00000000);
    auto reg_timgen = uio_acc.GetAccessor(0x00010000);
    auto reg_fmtr   = uio_acc.GetAccessor(0x00100000);
    auto reg_wdma0  = uio_acc.GetAccessor(0x00210000);
    auto reg_wdma1  = uio_acc.GetAccessor(0x00220000);

    std::cout << "CORE ID" << std::endl;
    std::cout << std::hex << reg_sys.ReadReg(SYSREG_ID) << std::endl;
    std::cout << std::hex << reg_timgen.ReadReg(TIMGENREG_CORE_ID) << std::endl;
    std::cout << std::hex << reg_fmtr.ReadReg(0) << std::endl;
    std::cout << std::hex << reg_wdma0.ReadReg(0) << std::endl;
    std::cout << std::hex << reg_wdma1.ReadReg(0) << std::endl;

    jelly::UdmabufAccessor udmabuf0_acc("udmabuf-jelly-vram0");
    if ( !udmabuf0_acc.IsMapped() ) {
        std::cout << "udmabuf0 mmap error" << std::endl;
        return 1;
    }
    auto dmabuf0_phys_adr = udmabuf0_acc.GetPhysAddr();
    auto dmabuf0_mem_size = udmabuf0_acc.GetSize();
    std::cout << "udmabuf0 phys addr : 0x" << std::hex << dmabuf0_phys_adr << std::endl;
    std::cout << "udmabuf0 size      : " << std::dec << dmabuf0_mem_size << std::endl;
    int rec_frames = dmabuf0_mem_size / (width * height * 2);
    std::cout << "udmabuf0 rec_frames : " << rec_frames << std::endl;

    jelly::UdmabufAccessor udmabuf1_acc("udmabuf-jelly-vram1");
    if ( !udmabuf1_acc.IsMapped() ) {
        std::cout << "udmabuf mmap error" << std::endl;
        return 1;
    }
    auto dmabuf1_phys_adr = udmabuf1_acc.GetPhysAddr();
    auto dmabuf1_mem_size = udmabuf1_acc.GetSize();
    std::cout << "udmabuf1 phys addr : 0x" << std::hex << dmabuf1_phys_adr << std::endl;
    std::cout << "udmabuf1 size      : " << std::dec << dmabuf1_mem_size << std::endl;

    rtcl::RtclP3S7ControlI2c cam;
    cam.Open("/dev/i2c-6", 0x10);

    reg_sys.WriteReg(SYSREG_CAM_ENABLE, 0);
    usleep(10000);
    reg_sys.WriteReg(SYSREG_CAM_ENABLE, 1);
    usleep(10000);

    std::cout << "Camera Module ID      : " << std::hex << cam.GetModuleId() << std::endl;
    std::cout << "Camera Module Version : " << std::hex << cam.GetModuleVersion() << std::endl;

    cam.SetDphySpeed(1250000000);
    std::cout << "Sensor PGood Enable : " << (pgood_enable ? "ON" : "OFF") << std::endl;
    cam.SetSensorPGoodEnable(pgood_enable);

    reg_sys.WriteReg(SYSREG_DPHY_SW_RESET, 1);

    std::cout << "Init Camera" << std::endl;
    cam.SetSensorPowerEnable(false);
    cam.SetDphyReset(true);
    usleep(10000);

    reg_sys.WriteReg(SYSREG_DPHY_SW_RESET, 0);
    usleep(10000);
    cam.SetCameraMode(rtcl::RtclP3S7ControlI2c::MODE_HIGH_SPEED);

    std::cout << "Sensor Power On" << std::endl;
    cam.SetSensorPowerEnable(true);
    usleep(10000);

    cam.SetDphyReset(false);
    if ( !cam.GetDphyInitDone() ) {
        std::cout << "!!ERROR!! CAM DPHY TX init_done = 0" << std::endl;
        return 1;
    }
    auto dphy_rx_init_done = reg_sys.ReadReg(SYSREG_DPHY_INIT_DONE);
    if ( dphy_rx_init_done == 0 ) {
        std::cout << "!!ERROR!! KV260 DPHY RX init_done = 0" << std::endl;
        return 1;
    }

    std::cout << "Sensor ID : " << cam.GetSensorId() << std::endl;
    reg_sys.WriteReg(SYSREG_IMAGE_WIDTH,  width);
    reg_sys.WriteReg(SYSREG_IMAGE_HEIGHT, height);
    reg_sys.WriteReg(SYSREG_BLACK_WIDTH,  1280);
    reg_sys.WriteReg(SYSREG_BLACK_HEIGHT, 1);

    auto xsm_delay = cam.CalcXsmDelay(width);
    cam.SetXsmDelay(xsm_delay);
    cam.SetNzrotXsmDelayEnable(true);
    cam.SetZeroRotEnable(true);

    if ( !cam.SetSensorEnable(true) ) {
        if ( !cam.GetSensorPGood() ) {
            std::cout << "\n!! sensor power good error. !! Retry with --pgood-off option." << std::endl;
        }
        else {
            std::cout << "!!ERROR!! CAM sensor enable failed" << std::endl;
        }
        cam.SetSensorPowerEnable(false);
        usleep(10000);
        reg_sys.WriteReg(SYSREG_CAM_ENABLE, 0);
        return 1;
    }

    cam.SetRoi0(width, height);
    reg_fmtr.WriteReg(REG_VIDEO_FMTREG_CTL_FRM_TIMER_EN,  1);
    reg_fmtr.WriteReg(REG_VIDEO_FMTREG_CTL_FRM_TIMEOUT,   20000000);
    reg_fmtr.WriteReg(REG_VIDEO_FMTREG_PARAM_WIDTH,       width);
    reg_fmtr.WriteReg(REG_VIDEO_FMTREG_PARAM_HEIGHT,      height);
    reg_fmtr.WriteReg(REG_VIDEO_FMTREG_PARAM_FILL,        0xffff);
    reg_fmtr.WriteReg(REG_VIDEO_FMTREG_PARAM_TIMEOUT,     100000);
    reg_fmtr.WriteReg(REG_VIDEO_FMTREG_CTL_CONTROL,       0x03);

    std::cout << "Start Camera" << std::endl;

    cam.SetMultTimer0(72);
    cam.SetFrLength0(0);
    cam.SetExposure0(10000);

    cam.SetTriggeredMode(true);
    cam.SetSlaveMode(true);
    cam.SetSequencerEnable(true);
    usleep(100000);

    cam.SetGainDb(10.0);

    jelly::VideoDmaControl vdmaw0(reg_wdma0, 2, 2, true);
    jelly::VideoDmaControl vdmaw1(reg_wdma1, 2, 2, true);
    std::deque<cv::Mat> frame_buffer;
    size_t buffer_frames = fps;   // 常に1秒分保持

    cv::namedWindow("img", cv::WINDOW_NORMAL);
    cv::resizeWindow("img", width + 64, height + 128);
    cv::imshow("img", cv::Mat::zeros(height, width, CV_8UC3));
    cv::setMouseCallback("img", on_mouse, nullptr);

    cv::createTrackbar("gain", "img", nullptr, 100);
    cv::setTrackbarPos("gain", "img", gain);
    cv::createTrackbar("fps", "img", nullptr, 1000);
    cv::setTrackbarMin("fps", "img", 5);
    cv::setTrackbarPos("fps", "img", fps);

    cv::createTrackbar("exposure", "img", nullptr, 90);
    cv::setTrackbarMin("exposure", "img", 10);
    cv::setTrackbarPos("exposure", "img", exposure);

    std::thread uart(uart_thread);
    uart.detach();

    int key;
    while ( (key = (cv::waitKey(10) & 0xff)) != 0x1b ) {
        if ( g_signal ) { break; }
        gain     = cv::getTrackbarPos("gain", "img");
        fps      = cv::getTrackbarPos("fps", "img");
        exposure = cv::getTrackbarPos("exposure", "img");

        // リングバッファは現在のFPSと同じ枚数（1秒分）
        buffer_frames = std::max(1, fps);

        cam.SetGainDb((float)gain / 10.0f);
        int period = 100000000 / fps;
        int trig_end = period * exposure / 100;
        reg_timgen.WriteReg(TIMGENREG_PARAM_PERIOD,      period-1);
        reg_timgen.WriteReg(TIMGENREG_PARAM_TRIG0_START, 1);
        reg_timgen.WriteReg(TIMGENREG_PARAM_TRIG0_END,   trig_end);
        reg_timgen.WriteReg(TIMGENREG_CTL_CONTROL, 3);

        vdmaw0.Oneshot(dmabuf0_phys_adr, width, height, 1);
        cv::Mat img_raw(height, width, CV_16U);
        udmabuf0_acc.MemCopyTo(img_raw.data, 0, width * height * 2);

        frame_buffer.push_back(img_raw.clone());
        while(frame_buffer.size() > buffer_frames)
        {
            frame_buffer.pop_front();
        }

        if(save_request.exchange(false))
        {
            // BLE受信時点までの1秒分を保存
            pre_buffer = frame_buffer;

            std::cout << "Saving previous 1 second..." << std::endl;

            int count = 0;
            for(auto &f : pre_buffer)
            {
                char fname[256];
                sprintf(fname, "rec/pre_%03d.png", count++);
                save_frame_png(f, fname, color);
            }

            std::cout << "Saved "
                      << pre_buffer.size()
                      << " pre-trigger frames." << std::endl;
        }

        cv::Mat img_preview = img_raw * 64;
        cv::Mat img_view;
        if ( color ) {
            cv::Mat img_bgr;
            cv::cvtColor(img_preview, img_view, cv::COLOR_BayerBG2BGR);
        }
        else {
            img_view = img_preview;
        }

        // --------------------------------------------------------
        // ホームベース前縁の計測結果を画面に表示
        // --------------------------------------------------------
        if (base_point_count >= 1)
        {
            cv::circle(img_view, base_point1, 5, cv::Scalar(0, 255, 0), -1);
        }

        if (base_point_count >= 2)
        {
            cv::circle(img_view, base_point2, 5, cv::Scalar(0, 255, 0), -1);
            cv::line(img_view, base_point1, base_point2,
                     cv::Scalar(0, 255, 0), 2);

            char text[128];
            sprintf(text, "Base edge: %.1f px", base_edge_pixels);
            cv::putText(img_view, text, cv::Point(10, 25),
                        cv::FONT_HERSHEY_SIMPLEX, 0.65,
                        cv::Scalar(0, 255, 0), 2);

            sprintf(text, "Target ball: %.1f px", target_ball_pixels);
            cv::putText(img_view, text, cv::Point(10, 50),
                        cv::FONT_HERSHEY_SIMPLEX, 0.65,
                        cv::Scalar(0, 255, 0), 2);
        }
        else
        {
            cv::putText(img_view,
                        "Click home-plate front edge: 2 points",
                        cv::Point(10, 25),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55,
                        cv::Scalar(0, 255, 0), 2);
        }

        cv::putText(img_view,
                    "Right click: reset",
                    cv::Point(10, height - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 255, 0), 1);

        cv::imshow("img", img_view);
    }

    std::cout << "close device" << std::endl;

    reg_fmtr.WriteReg(REG_VIDEO_FMTREG_CTL_CONTROL, 0x0);
    usleep(100000);

    cam.SetSequencerEnable(false);
    usleep(10000);
    cam.SetSensorEnable(false);
    usleep(10000);
    cam.SetSensorPowerEnable(false);
    usleep(10000);
    reg_sys.WriteReg(SYSREG_CAM_ENABLE, 0);
    usleep(100000);

    return 0;
}

void sensor_reg_dump(rtcl::RtclP3S7ControlI2c &cam, const char *fname) {
    FILE* fp = fopen(fname, "w");
    for ( int i = 0; i < 512; i++ ) {
        auto v = cam.spi_read(i);
        fprintf(fp, "%3d : 0x%04x (%d)\n", i, v, v);
    }
    fclose(fp);
}

void load_setting(rtcl::RtclP3S7ControlI2c &cam) {
    FILE* fp = fopen("reg_list.txt", "r");
    if ( fp == nullptr ) {
        std::cout << "reg_list.txt open error" << std::endl;
        return;
    }
    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        char *p = line;
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == '\0' || *p == '#') continue;
        unsigned int addr, data;
        int n = sscanf(p, "%i %i", &addr, &data);
        if (n == 2) {
            cam.spi_write((std::uint16_t)addr, (std::uint16_t)data);
        } else {
            std::cout << "parse error: " << line;
        }
    }
    fclose(fp);
}
