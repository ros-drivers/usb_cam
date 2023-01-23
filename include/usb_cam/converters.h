#ifndef USB_CAM_CONVERTERS_H
#define USB_CAM_CONVERTERS_H

#include "usb_cam/types.h"

namespace usb_cam
{

namespace util
{

namespace converters
{
/* Helper functions and table converters */
io_method_t io_method_from_string(const std::string & str);
pixel_format_t pixel_format_from_string(const std::string & str);
std::string pixel_format_to_string(const uint32_t & pixelformat);
color_format_t color_format_from_string(const std::string & str);
unsigned int v4l_pixel_format_from_pixel_format(const pixel_format_t& pixelformat, bool& mono);
std::string v4l_control_name_to_param_name(const char *name);

/* Standalone format converters */
/**
 * Conversion from YUV to RGB.
 * The normal conversion matrix is due to Julien (surname unknown):
 *
 * [ R ]   [  1.0   0.0     1.403 ] [ Y ]
 * [ G ] = [  1.0  -0.344  -0.714 ] [ U ]
 * [ B ]   [  1.0   1.770   0.0   ] [ V ]
 *
 * and the firewire one is similar:
 *
 * [ R ]   [  1.0   0.0     0.700 ] [ Y ]
 * [ G ] = [  1.0  -0.198  -0.291 ] [ U ]
 * [ B ]   [  1.0   1.015   0.0   ] [ V ]
 *
 * Corrected by BJT (coriander's transforms RGB->YUV and YUV->RGB
 *                   do not get you back to the same RGB!)
 * [ R ]   [  1.0   0.0     1.136 ] [ Y ]
 * [ G ] = [  1.0  -0.396  -0.578 ] [ U ]
 * [ B ]   [  1.0   2.041   0.002 ] [ V ]
 *
 */
bool YUV2RGB(const unsigned char & y,
             const unsigned char & u,
             const unsigned char & v,
             unsigned char * r,
             unsigned char * g,
             unsigned char * b);
bool MONO102MONO8(const char * RAW,
                  char * & MONO,
                  const int & NumPixels);
bool YUYV2RGB(const char * YUV,
              char * & RGB,
              const int & NumPixels);
bool COPY2RGB(const char * input,
              char * & output,
              const int & NumPixels);
bool YUV4202RGB(char * YUV,
                char * & RGB,
                const int & width,
                const int & height);
std::string FCC2S(const unsigned int & val);
bool UYVY2RGB(const char * YUV,
              char * & RGB,
              const int & NumPixels);

}

}

}

#endif // USB_CAM_CONVERTERS_H
