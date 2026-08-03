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

std::atomic<bool> save_request(false);

bool capturing_after_trigger = false;

std::deque<cv::Mat> after_buffer;
std::deque<cv::Mat> pre_buffer;

size_t after_target_frames = 0;

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


void uart_thread()
{
    int fd = open("/dev/ttyUSB1", O_RDONLY | O_NOCTTY);

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

    // ソフトウェアフロー制御OFF
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // 完全RAWモード
    tty.c_lflag = 0;
    tty.c_iflag &= ~(ICRNL | INLCR | IGNCR);
    tty.c_oflag = 0;

    // 1バイト来たらすぐread()を返す
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

    // set signal
    signal(SIGINT, signal_handler);

    // mmap uio
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
    
    // レジスタ確認
    std::cout << "CORE ID" << std::endl;
    std::cout << std::hex << reg_sys.ReadReg(SYSREG_ID) << std::endl;
    std::cout << std::hex << reg_timgen.ReadReg(TIMGENREG_CORE_ID) << std::endl;
    std::cout << std::hex << reg_fmtr.ReadReg(0) << std::endl;
    std::cout << std::hex << reg_wdma0.ReadReg(0) << std::endl;
    std::cout << std::hex << reg_wdma1.ReadReg(0) << std::endl;

    // mmap udmabuf0
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

    // mmap udmabuf1
    jelly::UdmabufAccessor udmabuf1_acc("udmabuf-jelly-vram1");
    if ( !udmabuf1_acc.IsMapped() ) {
        std::cout << "udmabuf mmap error" << std::endl;
        return 1;
    }
    auto dmabuf1_phys_adr = udmabuf1_acc.GetPhysAddr();
    auto dmabuf1_mem_size = udmabuf1_acc.GetSize();
    std::cout << "udmabuf1 phys addr : 0x" << std::hex << dmabuf1_phys_adr << std::endl;
    std::cout << "udmabuf1 size      : " << std::dec << dmabuf1_mem_size << std::endl;

    // カメラ制御生成
    rtcl::RtclP3S7ControlI2c cam;
    cam.Open("/dev/i2c-6", 0x10);

    // カメラモジュールリセット
    reg_sys.WriteReg(SYSREG_CAM_ENABLE, 0);
    usleep(10000);
    reg_sys.WriteReg(SYSREG_CAM_ENABLE, 1);
    usleep(10000);

    // カメラ基板ID確認
    std::cout << "Camera Module ID      : " << std::hex << cam.GetModuleId() << std::endl;
    std::cout << "Camera Module Version : " << std::hex << cam.GetModuleVersion() << std::endl;

    // MMCM 設定
    cam.SetDphySpeed(1250000000);   // 1250Mbps

    // センサー電源OK監視有無設定
    std::cout << "Sensor PGood Enable : " << (pgood_enable ? "ON" : "OFF") << std::endl;
    cam.SetSensorPGoodEnable(pgood_enable);

    // 受信側 DPHY リセット
    reg_sys.WriteReg(SYSREG_DPHY_SW_RESET, 1);

    // カメラ基板初期化
    std::cout << "Init Camera" << std::endl;
    cam.SetSensorPowerEnable(false);
    cam.SetDphyReset(true);
    usleep(10000);

    // 受信側 DPHY 解除 (必ずこちらを先に解除)
    reg_sys.WriteReg(SYSREG_DPHY_SW_RESET, 0);
    usleep(10000);

    // 高速モード設定
    cam.SetCameraMode(rtcl::RtclP3S7ControlI2c::MODE_HIGH_SPEED);

    // センサー電源ON
    std::cout << "Sensor Power On" << std::endl;
    cam.SetSensorPowerEnable(true);
    usleep(10000);

    // センサー基板 DPHY-TX リセット解除
    cam.SetDphyReset(false);
    if ( !cam.GetDphyInitDone() ) {
        std::cout << "!!ERROR!! CAM DPHY TX init_done = 0" << std::endl;
        return 1;
    }

    // ここで RX 側も init_done が来る
    auto dphy_rx_init_done = reg_sys.ReadReg(SYSREG_DPHY_INIT_DONE);
    if ( dphy_rx_init_done == 0 ) {
        std::cout << "!!ERROR!! KV260 DPHY RX init_done = 0" << std::endl;
        return 1;
    }

    // センサーID確認
    std::cout << "Sensor ID : " << cam.GetSensorId() << std::endl;

    // 受信画像サイズ設定
    reg_sys.WriteReg(SYSREG_IMAGE_WIDTH,  width);
    reg_sys.WriteReg(SYSREG_IMAGE_HEIGHT, height);
    reg_sys.WriteReg(SYSREG_BLACK_WIDTH,  1280);
    reg_sys.WriteReg(SYSREG_BLACK_HEIGHT, 1);

    // D-PHY速度とセンサー速度の差に対して、各ラインの追加ディレイ(xsm-delay) を計算して設定
    auto xsm_delay = cam.CalcXsmDelay(width);
    cam.SetXsmDelay(xsm_delay);
    cam.SetNzrotXsmDelayEnable(true);
    cam.SetZeroRotEnable(true);

    // センサー起動
    if ( !cam.SetSensorEnable(true) ) {
        if ( !cam.GetSensorPGood() ) {
            std::cout << "\n!! sensor power good error. !! Retry with --pgood-off option." << std::endl;
        }
        else {
            std::cout << "!!ERROR!! CAM sensor enable failed" << std::endl;
        }
        // カメラモジュールOFF
        cam.SetSensorPowerEnable(false);
        usleep(10000);
        reg_sys.WriteReg(SYSREG_CAM_ENABLE, 0);
        return 1;
    }

    // 画像サイズ設定
    cam.SetRoi0(width, height);

    // video input start
    reg_fmtr.WriteReg(REG_VIDEO_FMTREG_CTL_FRM_TIMER_EN,  1);
    reg_fmtr.WriteReg(REG_VIDEO_FMTREG_CTL_FRM_TIMEOUT,   20000000);
    reg_fmtr.WriteReg(REG_VIDEO_FMTREG_PARAM_WIDTH,       width);
    reg_fmtr.WriteReg(REG_VIDEO_FMTREG_PARAM_HEIGHT,      height);
    reg_fmtr.WriteReg(REG_VIDEO_FMTREG_PARAM_FILL,        0xffff);
    reg_fmtr.WriteReg(REG_VIDEO_FMTREG_PARAM_TIMEOUT,     100000);
    reg_fmtr.WriteReg(REG_VIDEO_FMTREG_CTL_CONTROL,       0x03);

    // 動作開始
    std::cout << "Start Camera" << std::endl;

    cam.SetMultTimer0(72);
    cam.SetFrLength0(0);
    cam.SetExposure0(10000);

    cam.SetTriggeredMode(true);
    cam.SetSlaveMode(true);
    cam.SetSequencerEnable(true);
    usleep(100000);

    cam.SetGainDb(10.0);

    // Video DMA ドライバ生成
    jelly::VideoDmaControl vdmaw0(reg_wdma0, 2, 2, true);
    jelly::VideoDmaControl vdmaw1(reg_wdma1, 2, 2, true);

    std::deque<cv::Mat> frame_buffer;
    //const size_t buffer_frames = fps / 2;
    const size_t buffer_frames =100;

    cv::namedWindow("img", cv::WINDOW_NORMAL);
    cv::resizeWindow("img", width + 64, height + 128);
    cv::imshow("img", cv::Mat::zeros(height, width, CV_8UC3));
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

    int     key;
    while ( (key = (cv::waitKey(10) & 0xff)) != 0x1b ) {
        if ( g_signal ) { break; }

        gain     = cv::getTrackbarPos("gain", "img");
        fps      = cv::getTrackbarPos("fps", "img");
        exposure = cv::getTrackbarPos("exposure", "img");

        cam.SetGainDb((float)gain / 10.0f);

        int period = 100000000 / fps;   // 100MHz / fps
        int trig_end = period * exposure / 100;
        reg_timgen.WriteReg(TIMGENREG_PARAM_PERIOD,      period-1);
        reg_timgen.WriteReg(TIMGENREG_PARAM_TRIG0_START, 1);
        reg_timgen.WriteReg(TIMGENREG_PARAM_TRIG0_END,   trig_end);
        reg_timgen.WriteReg(TIMGENREG_CTL_CONTROL, 3);

        // 画像読み込み
        vdmaw0.Oneshot(dmabuf0_phys_adr, width, height, 1);
        cv::Mat img_raw(height, width, CV_16U);
        udmabuf0_acc.MemCopyTo(img_raw.data, 0, width * height * 2);

        // リングバッファへ追加
        frame_buffer.push_back(img_raw.clone());

        while(frame_buffer.size() > buffer_frames)
	{
    		frame_buffer.pop_front();
	}
	//----------------------------------
	// SAVE受信
	//----------------------------------

	if(save_request.exchange(false))
	{
    		capturing_after_trigger = true;

		pre_buffer = frame_buffer;

    		after_buffer.clear();

    		after_target_frames = buffer_frames;

    		std::cout << "Capture after trigger..." << std::endl;
	}
        
	// 0.5秒後にSAVE
	if(capturing_after_trigger)
	{
		after_buffer.push_back(img_raw.clone());
	
		if(after_buffer.size() >= after_target_frames)
    		{
			capturing_after_trigger = false;

    			std::cout << "Saving frames..." << std::endl;

    			int count = 0;

			//-------------------------------------------------
        		// 前0.5秒
        		//-------------------------------------------------

    			for(auto &f : pre_buffer)
    			{
        			char fname[256];

        			sprintf(fname,
                		"rec/pre_%03d.png",
                		count++);

        			cv::imwrite(fname,
                    		f * (65535.0/1023.0));
    			}
			//-------------------------------------------------
		        // 後0.5秒
        		//-------------------------------------------------

        		count = 0;

        		for(auto &f : after_buffer)
        		{
            			char fname[256];

            			sprintf(fname,
                    		"rec/post_%04d.png",
                    		count++);

            			cv::imwrite(fname,
                        	f * (65535.0/1023.0));
        		}
	
    			std::cout << "Saved "
              		<< frame_buffer.size() + after_buffer.size()
              		<< " frames"
              		<< std::endl;
		}
	}
    
        // 表示画像準備
        cv::Mat img_preview = img_raw * 64 ; // 10bit -> 16bit
	cv::Mat img_view;

        if ( color ) {
            cv::Mat img_bgr;
            cv::cvtColor(img_preview, img_view, cv::COLOR_BayerBG2BGR);
        }
        else {
            img_view = img_preview;
        }

	// 表示
        cv::imshow("img", img_view);
    }
	
    std::cout << "close device" << std::endl;

    // video input stop
    reg_fmtr.WriteReg(REG_VIDEO_FMTREG_CTL_CONTROL, 0x0);
    usleep(100000);

    // シーケンサ停止
    cam.SetSequencerEnable(false);
    usleep(10000);

    // センサー停止
    cam.SetSensorEnable(false);
    usleep(10000);

    // センサー電源OFF
    cam.SetSensorPowerEnable(false);
    usleep(10000);

    // カメラモジュールOFF
    reg_sys.WriteReg(SYSREG_CAM_ENABLE, 0);
    usleep(100000);

    return 0;
}


// センサーのレジスタダンプ
void sensor_reg_dump(rtcl::RtclP3S7ControlI2c &cam, const char *fname) {
    FILE* fp = fopen(fname, "w");
    for ( int i = 0; i < 512; i++ ) {
        auto v = cam.spi_read(i);
        fprintf(fp, "%3d : 0x%04x (%d)\n", i, v, v);
    }
    fclose(fp);
}

// 設定ファイルを読み込む
void load_setting(rtcl::RtclP3S7ControlI2c &cam) {
    FILE* fp = fopen("reg_list.txt", "r");
    if ( fp == nullptr ) {
        std::cout << "reg_list.txt open error" << std::endl;
        return;
    }
    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        char *p = line;
        // skip leading whitespace
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == '\0' || *p == '#') continue; // skip empty/comment
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

// end of file
