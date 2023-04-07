#ifndef USB_CAM_TYPES_H
#define USB_CAM_TYPES_H

#include <vector>
#include <string>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <ctime>
#include <climits>
#include <cstring>
#include <stdexcept>

#include <asm/types.h> // for videodev2.h
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include <opencv2/imgproc.hpp>

namespace usb_cam
{
namespace constants
{

static const std::vector<unsigned char> uchar_clipping_table = {
    0, 0, 0, 0, 0, 0, 0, 0,        // -128 - -121
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // -120 - -101
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // -100 - -81
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // -80  - -61
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // -60  - -41
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // -40  - -21
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // -20  - -1
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
    61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80,
    81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100,
    101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115,
    116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130,
    131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145,
    146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160,
    161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175,
    176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190,
    191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205,
    206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220,
    221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235,
    236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250,
    251, 252, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255,  // 256-263
    255, 255, 255, 255, 255, 255, 255, 255,  // 264-271
    255, 255, 255, 255, 255, 255, 255, 255,  // 272-279
    255, 255, 255, 255, 255, 255, 255, 255,  // 280-287
    255, 255, 255, 255, 255, 255, 255, 255,  // 288-295
    255, 255, 255, 255, 255, 255, 255, 255,  // 296-303
    255, 255, 255, 255, 255, 255, 255, 255,  // 304-311
    255, 255, 255, 255, 255, 255, 255, 255,  // 312-319
    255, 255, 255, 255, 255, 255, 255, 255,  // 320-327
    255, 255, 255, 255, 255, 255, 255, 255,  // 328-335
    255, 255, 255, 255, 255, 255, 255, 255,  // 336-343
    255, 255, 255, 255, 255, 255, 255, 255,  // 344-351
    255, 255, 255, 255, 255, 255, 255, 255,  // 352-359
    255, 255, 255, 255, 255, 255, 255, 255,  // 360-367
    255, 255, 255, 255, 255, 255, 255, 255,  // 368-375
    255, 255, 255, 255, 255, 255, 255, 255,  // 376-383
};

static const int clipping_table_offset = 128;

}  // namespace constants

enum io_method_t
{
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
    IO_METHOD_UNKNOWN,
    };


enum pixel_format_t
{
    PIXEL_FORMAT_YUYV,
    PIXEL_FORMAT_UYVY,
    PIXEL_FORMAT_MJPEG,
    PIXEL_FORMAT_YUVMONO10,
    PIXEL_FORMAT_RGB24,
    PIXEL_FORMAT_BGR24,
    PIXEL_FORMAT_GREY,
    PIXEL_FORMAT_YU12,
    PIXEL_FORMAT_H264,
    PIXEL_FORMAT_UNKNOWN
};


enum color_format_t
{
    COLOR_FORMAT_YUV420P,
    COLOR_FORMAT_YUV422P,
    COLOR_FORMAT_UNKNOWN,
};


struct buffer
{
    void * start;
    std::size_t length;
};

struct camera_image_t
{
    uint32_t width;
    uint32_t height;
    uint32_t step;
    std::string encoding;
    int bytes_per_pixel;
    int image_size;
    struct timespec stamp;
    char * image;
    int is_new;
};

struct capture_format_t
{
    struct v4l2_fmtdesc format;
    struct v4l2_frmsizeenum size;
    struct v4l2_frmivalenum interval;
};

struct camera_control_t
{
    v4l2_ctrl_type type;
    std::string name;
    std::string description;
    std::string value;
};

}  // namespace usb_cam

#endif  // USB_CAM_TYPES_H
