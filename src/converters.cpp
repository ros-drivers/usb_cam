#include "usb_cam/converters.h"
#include "usb_cam/types.h"
#include "usb_cam/util.h"
#include <linux/videodev2.h>

namespace usb_cam
{

namespace util
{

namespace converters
{

io_method_t io_method_from_string(const std::string & str)
{
    if (str == "mmap")
        return io_method_t::IO_METHOD_MMAP;
    else if (str == "read")
        return io_method_t::IO_METHOD_READ;
    else if (str == "userptr")
        return io_method_t::IO_METHOD_USERPTR;
    else
        return io_method_t::IO_METHOD_UNKNOWN;
}

pixel_format_t pixel_format_from_string(const std::string & str)
{
    if ((str == "yuyv") || (str == "yuv"))
        return pixel_format_t::PIXEL_FORMAT_YUYV;
    else if (str == "uyvy")
        return pixel_format_t::PIXEL_FORMAT_UYVY;
    else if (str == "mjpeg")
        return pixel_format_t::PIXEL_FORMAT_MJPEG;
    else if (str == "h264")
        return pixel_format_t::PIXEL_FORMAT_H264;
    else if (str == "yuvmono10")
        return pixel_format_t::PIXEL_FORMAT_YUVMONO10;
    else if (str == "rgb24")
        return pixel_format_t::PIXEL_FORMAT_RGB24;
    else if (str == "grey")
        return pixel_format_t::PIXEL_FORMAT_GREY;
    else if (str == "yu12")
        return pixel_format_t::PIXEL_FORMAT_YU12;
    else if (str == "bgr24")
        return PIXEL_FORMAT_BGR24;
    else
        return pixel_format_t::PIXEL_FORMAT_UNKNOWN;
}

std::string pixel_format_to_string(const uint32_t & pixelformat)
{
    switch (pixelformat)
    {
    case pixel_format_t::PIXEL_FORMAT_YUYV:
        return "yuyv";
    case pixel_format_t::PIXEL_FORMAT_UYVY:
        return "uyvy";
    case pixel_format_t::PIXEL_FORMAT_MJPEG:
        return "mjpeg";
    case pixel_format_t::PIXEL_FORMAT_H264:
        return "h264";
    case pixel_format_t::PIXEL_FORMAT_YUVMONO10:
        return "yuvmono10";
    case pixel_format_t::PIXEL_FORMAT_RGB24:
        return "rgb24";
    case pixel_format_t::PIXEL_FORMAT_BGR24:
        return "bgr24";
    case pixel_format_t::PIXEL_FORMAT_GREY:
        return "grey";
    case pixel_format_t::PIXEL_FORMAT_YU12:
        return "yu12";
    case pixel_format_t::PIXEL_FORMAT_UNKNOWN:
    default:
        return "unknown";
    }
}

color_format_t color_format_from_string(const std::string & str)
{
    if (str == "yuv420p")
        return color_format_t::COLOR_FORMAT_YUV420P;
    else if (str == "yuv422p")
        return color_format_t::COLOR_FORMAT_YUV422P;
    else
        return color_format_t::COLOR_FORMAT_UNKNOWN;
}


unsigned int v4l_pixel_format_from_pixel_format(const pixel_format_t &pixelformat, bool &mono)
{
    mono = false;
    switch(pixelformat)
    {
    case PIXEL_FORMAT_YUYV:
        return V4L2_PIX_FMT_YUYV;
    case PIXEL_FORMAT_UYVY:
        return V4L2_PIX_FMT_UYVY;
    case PIXEL_FORMAT_MJPEG:
        return V4L2_PIX_FMT_MJPEG;
    case PIXEL_FORMAT_H264:
        return V4L2_PIX_FMT_H264;
    case PIXEL_FORMAT_YUVMONO10:
        mono = true;
        return V4L2_PIX_FMT_YUYV;
    case PIXEL_FORMAT_RGB24:
        return V4L2_PIX_FMT_RGB24;
    case PIXEL_FORMAT_BGR24:
        return V4L2_PIX_FMT_BGR24;
    case PIXEL_FORMAT_GREY:
        mono = true;
        return V4L2_PIX_FMT_GREY;
    case PIXEL_FORMAT_YU12:
        return V4L2_PIX_FMT_YUV420;
    default:
        return UINT_MAX;
    }
}

bool YUV2RGB(const unsigned char &y,
             const unsigned char &u,
             const unsigned char &v,
             unsigned char *r,
             unsigned char *g,
             unsigned char *b)
{
    const int y2 = static_cast<int>(y);
    const int u2 = static_cast<int>(u - 128);
    const int v2 = static_cast<int>(v - 128);
    // std::cerr << "YUV=("<<y2<<","<<u2<<","<<v2<<")"<<std::endl;

    // This is the normal YUV conversion, but
    // appears to be incorrect for the firewire cameras
    //   int r2 = y2 + ( (v2*91947) >> 16);
    //   int g2 = y2 - ( ((u2*22544) + (v2*46793)) >> 16 );
    //   int b2 = y2 + ( (u2*115999) >> 16);
    // This is an adjusted version (UV spread out a bit)
    int r2 = y2 + ((v2 * 37221) >> 15);
    int g2 = y2 - (((u2 * 12975) + (v2 * 18949)) >> 15);
    int b2 = y2 + ((u2 * 66883) >> 15);
    // std::cerr << "   RGB=("<<r2<<","<<g2<<","<<b2<<")"<<std::endl;

    // Cap the values.
    *r = util::CLIPVALUE(r2);
    *g = util::CLIPVALUE(g2);
    *b = util::CLIPVALUE(b2);

    return true;
}

bool MONO102MONO8(const char *RAW, char *&MONO, const int &NumPixels)
{
    int i, j;
    for (i = 0, j = 0; i < (NumPixels << 1); i += 2, j += 1)
    {
        // first byte is low byte, second byte is high byte; smash together and convert to 8-bit
        MONO[j] = (unsigned char)(((RAW[i + 0] >> 2) & 0x3F) | ((RAW[i + 1] << 6) & 0xC0));
    }
    return true;
}

bool YUYV2RGB(const char *YUV, char *&RGB, const int &NumPixels)
{
    int i, j;
    unsigned char y0, y1, u, v;
    unsigned char r, g, b;

    for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
    {
        y0 = (unsigned char)YUV[i + 0];
        u = (unsigned char)YUV[i + 1];
        y1 = (unsigned char)YUV[i + 2];
        v = (unsigned char)YUV[i + 3];
        YUV2RGB(y0, u, v, &r, &g, &b);
        RGB[j + 0] = r;
        RGB[j + 1] = g;
        RGB[j + 2] = b;
        YUV2RGB(y1, u, v, &r, &g, &b);
        RGB[j + 3] = r;
        RGB[j + 4] = g;
        RGB[j + 5] = b;
    }
    return true;
}

bool COPY2RGB(const char *input, char *&output, const int &NumPixels)
{
    memcpy(output, input, NumPixels * 3);
    return true;
}

bool YUV4202RGB(char *YUV, char *&RGB, const int &width, const int &height)
{
    cv::Size size(height, width);
    cv::Mat cv_img(height * 1.5, width, CV_8UC1, YUV);
    cv::Mat cv_out(height, width, CV_8UC3, RGB);
    cvtColor(cv_img, cv_out, cv::COLOR_YUV420p2BGR);
    return true;
}

std::string FCC2S(const unsigned int &val)
{
    std::string s;

    s += val & 0x7f;
    s += (val >> 8) & 0x7f;
    s += (val >> 16) & 0x7f;
    s += (val >> 24) & 0x7f;
    if (val & (1 << 31)) {
        s += "-BE";
    }
    return s;
}

bool UYVY2RGB(const char *YUV, char *&RGB, const int &NumPixels)
{
    int i, j;
    unsigned char y0, y1, u, v;
    unsigned char r, g, b;
    for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6) {
        u = (unsigned char)YUV[i + 0];
        y0 = (unsigned char)YUV[i + 1];
        v = (unsigned char)YUV[i + 2];
        y1 = (unsigned char)YUV[i + 3];
        YUV2RGB(y0, u, v, &r, &g, &b);
        RGB[j + 0] = r;
        RGB[j + 1] = g;
        RGB[j + 2] = b;
        YUV2RGB(y1, u, v, &r, &g, &b);
        RGB[j + 3] = r;
        RGB[j + 4] = g;
        RGB[j + 5] = b;
    }
    return true;
}

std::string v4l_control_name_to_param_name(const char *name)
{ // https://github.com/cz172638/v4l-utils/blob/master/utils/v4l2-ctl/v4l2-ctl-common.cpp
    std::string s;
    int add_underscore = 0;

    while (*name) {
        if (isalnum(*name)) {
            if (add_underscore)
                s += '_';
            add_underscore = 0;
            s += std::string(1, tolower(*name));
        }
        else if (s.length()) add_underscore = 1;
        name++;
    }
    return s;
}

}

}

}
