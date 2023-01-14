// Copyright 2014 Robert Bosch, LLC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Robert Bosch, LLC nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef USB_CAM__UTILS_HPP_
#define USB_CAM__UTILS_HPP_

#include <sys/ioctl.h>
#include <sys/time.h>
#include <cmath>
#include <ctime>
#include <cstring>
#include <sstream>
#include <string>

#include "usb_cam/constants.hpp"


namespace usb_cam
{
namespace utils
{


typedef enum
{
  IO_METHOD_READ,
  IO_METHOD_MMAP,
  IO_METHOD_USERPTR,
  IO_METHOD_UNKNOWN,
} io_method_t;


typedef enum
{
  PIXEL_FORMAT_YUYV,
  PIXEL_FORMAT_UYVY,
  PIXEL_FORMAT_MJPEG,
  PIXEL_FORMAT_YUVMONO10,
  PIXEL_FORMAT_RGB24,
  PIXEL_FORMAT_GREY,
  PIXEL_FORMAT_YU12,
  PIXEL_FORMAT_H264,
  PIXEL_FORMAT_UNKNOWN
} pixel_format_t;


typedef enum
{
  COLOR_FORMAT_YUV420P,
  COLOR_FORMAT_YUV422P,
  COLOR_FORMAT_UNKNOWN,
} color_format_t;


struct buffer
{
  void * start;
  size_t length;
};


/// @brief Get epoch time shift
/// @details Run this at start of process to calculate epoch time shift
/// @ref https://stackoverflow.com/questions/10266451/where-does-v4l2-buffer-timestamp-value-starts-counting
inline time_t get_epoch_time_shift()
{
  struct timeval epoch_time;
  struct timespec monotonic_time;

  gettimeofday(&epoch_time, NULL);
  clock_gettime(CLOCK_MONOTONIC, &monotonic_time);

  const int64_t uptime_ms =
    monotonic_time.tv_sec * 1000 + static_cast<int64_t>(
    std::round(monotonic_time.tv_nsec / 1000000.0));
  const int64_t epoch_ms =
    epoch_time.tv_sec * 1000 + static_cast<int64_t>(
    std::round(epoch_time.tv_usec / 1000.0));

  return static_cast<time_t>((epoch_ms - uptime_ms) / 1000);
}


inline int xioctl(int fd, int request, void * arg)
{
  int r;

  do {
    r = ioctl(fd, request, arg);
    continue;
  } while (-1 == r && EINTR == errno);

  return r;
}

/** Clip a value to the range 0<val<255. For speed this is done using an
 * array, so can only cope with numbers in the range -128<=val<=383.
 */
inline unsigned char CLIPVALUE(const int & val)
{
  // Old method (if)
  /*   val = val < 0 ? 0 : val; */
  /*   return val > 255 ? 255 : val; */

  try {
    // New method array
    return usb_cam::constants::uchar_clipping_table.at(
      val + usb_cam::constants::clipping_table_offset);
  } catch (std::out_of_range const &) {
    // fall back to old method
    unsigned char clipped_val = val < 0 ? 0 : static_cast<unsigned char>(val);
    return val > 255 ? 255 : clipped_val;
  }
}


inline io_method_t io_method_from_string(const std::string & str)
{
  if (str == "mmap") {
    return io_method_t::IO_METHOD_MMAP;
  } else if (str == "read") {
    return io_method_t::IO_METHOD_READ;
  } else if (str == "userptr") {
    return io_method_t::IO_METHOD_USERPTR;
  } else {
    return io_method_t::IO_METHOD_UNKNOWN;
  }
}


inline pixel_format_t pixel_format_from_string(const std::string & str)
{
  if (str == "yuyv") {
    return pixel_format_t::PIXEL_FORMAT_YUYV;
  } else if (str == "uyvy") {
    return pixel_format_t::PIXEL_FORMAT_UYVY;
  } else if (str == "mjpeg") {
    return pixel_format_t::PIXEL_FORMAT_MJPEG;
  } else if (str == "h264") {
    return pixel_format_t::PIXEL_FORMAT_H264;
  } else if (str == "yuvmono10") {
    return pixel_format_t::PIXEL_FORMAT_YUVMONO10;
  } else if (str == "rgb24") {
    return pixel_format_t::PIXEL_FORMAT_RGB24;
  } else if (str == "grey") {
    return pixel_format_t::PIXEL_FORMAT_GREY;
  } else if (str == "yu12") {
    return pixel_format_t::PIXEL_FORMAT_YU12;
  } else {
    return pixel_format_t::PIXEL_FORMAT_UNKNOWN;
  }
}


inline std::string pixel_format_to_string(const uint32_t & pixelformat)
{
  switch (pixelformat) {
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
    case pixel_format_t::PIXEL_FORMAT_GREY:
      return "grey";
    case pixel_format_t::PIXEL_FORMAT_YU12:
      return "yu12";
    case pixel_format_t::PIXEL_FORMAT_UNKNOWN:
    default:
      return "unknown";
  }
}


inline color_format_t color_format_from_string(const std::string & str)
{
  if (str == "yuv420p") {
    return color_format_t::COLOR_FORMAT_YUV420P;
  } else if (str == "yuv422p") {
    return color_format_t::COLOR_FORMAT_YUV422P;
  } else {
    return color_format_t::COLOR_FORMAT_UNKNOWN;
  }
}

}  // namespace utils
}  // namespace usb_cam

#endif  // USB_CAM__UTILS_HPP_
