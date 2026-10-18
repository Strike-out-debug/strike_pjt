
#include "CameraCapture.h"

#include "jelly/UioAccessor.h"
#include "jelly/UdmabufAccessor.h"
#include "jelly/JellyRegs.h"
#include "jelly/I2cAccessor.h"
#include "jelly/GpioAccessor.h"
#include "jelly/VideoDmaControl.h"
#include "rtcl/RtclP3S7Control.h"

struct CameraCapture::Impl {
    int width = 640;
    int height = 480;
    int fps = 200;
    int exposure = 90;
    double gain_db = 0.0;
    bool opened = false;
};
