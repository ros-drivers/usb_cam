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

#include "usb_cam/constants.hpp"

#include <sys/ioctl.h>

#include <cstring>
#include <sstream>
#include <string>

#include "opencv2/opencv.hpp"


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
} io_method;


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
} pixel_format;


typedef enum
{
  COLOR_FORMAT_YUV420P,
  COLOR_FORMAT_YUV422P,
  COLOR_FORMAT_UNKNOWN,
} color_format;


struct buffer
{
  void * start;
  size_t length;
};


void monotonicToRealTime(const timespec & monotonic_time, timespec & real_time)
{
  struct timespec real_sample1, real_sample2, monotonic_sample;

  // TODO(lucasw) Disable interrupts here?
  // otherwise what if there is a delay/interruption between sampling the times?
  clock_gettime(CLOCK_REALTIME, &real_sample1);
  clock_gettime(CLOCK_MONOTONIC, &monotonic_sample);
  clock_gettime(CLOCK_REALTIME, &real_sample2);

  timespec time_diff;
  time_diff.tv_sec = real_sample2.tv_sec - monotonic_sample.tv_sec;
  time_diff.tv_nsec = real_sample2.tv_nsec - monotonic_sample.tv_nsec;

  // This isn't available outside of the kernel
  // real_time = timespec_add(monotonic_time, time_diff);

  const int64_t NSEC_PER_SEC = 1000000000;
  real_time.tv_sec = monotonic_time.tv_sec + time_diff.tv_sec;
  real_time.tv_nsec = monotonic_time.tv_nsec + time_diff.tv_nsec;
  if (real_time.tv_nsec >= NSEC_PER_SEC) {
    ++real_time.tv_sec;
    real_time.tv_nsec -= NSEC_PER_SEC;
  } else if (real_time.tv_nsec < 0) {
    --real_time.tv_sec;
    real_time.tv_nsec += NSEC_PER_SEC;
  }
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
 * array, so can only cope with numbers in the range -128<val<383.
 */
inline unsigned char CLIPVALUE(const int & val)
{
  // Old method (if)
  /*   val = val < 0 ? 0 : val; */
  /*   return val > 255 ? 255 : val; */

  // New method (array)
  return usb_cam::constants::uchar_clipping_table[val + usb_cam::constants::clipping_table_offset];
}


inline io_method io_method_from_string(const std::string & str)
{
  if (str == "mmap") {
    return io_method::IO_METHOD_MMAP;
  } else if (str == "read") {
    return io_method::IO_METHOD_READ;
  } else if (str == "userptr") {
    return io_method::IO_METHOD_USERPTR;
  } else {
    return io_method::IO_METHOD_UNKNOWN;
  }
}


inline pixel_format pixel_format_from_string(const std::string & str)
{
  if (str == "yuyv") {
    return PIXEL_FORMAT_YUYV;
  } else if (str == "uyvy") {
    return PIXEL_FORMAT_UYVY;
  } else if (str == "mjpeg") {
    return PIXEL_FORMAT_MJPEG;
  } else if (str == "h264") {
    return PIXEL_FORMAT_H264;
  } else if (str == "yuvmono10") {
    return PIXEL_FORMAT_YUVMONO10;
  } else if (str == "rgb24") {
    return PIXEL_FORMAT_RGB24;
  } else if (str == "grey") {
    return PIXEL_FORMAT_GREY;
  } else if (str == "yu12") {
    return PIXEL_FORMAT_YU12;
  } else {
    return PIXEL_FORMAT_UNKNOWN;
  }
}


inline std::string pixel_format_to_string(const uint32_t & pixelformat)
{
  switch (pixelformat) {
    case PIXEL_FORMAT_YUYV:
      return "yuyv";
    case PIXEL_FORMAT_UYVY:
      return "uyvy";
    case PIXEL_FORMAT_MJPEG:
      return "mjpeg";
    case PIXEL_FORMAT_H264:
      return "h264";
    case PIXEL_FORMAT_YUVMONO10:
      return "yuvmono10";
    case PIXEL_FORMAT_RGB24:
      return "rgb24";
    case PIXEL_FORMAT_GREY:
      return "grey";
    case PIXEL_FORMAT_YU12:
      return "yu12";
    case PIXEL_FORMAT_UNKNOWN:
    default:
      return "unknown";
  }
}


color_format color_format_from_string(const std::string & str)
{
  if (str == "yuv420p") {
    return COLOR_FORMAT_YUV420P;
  } else if (str == "yuv422p") {
    return COLOR_FORMAT_YUV422P;
  } else {
    return COLOR_FORMAT_UNKNOWN;
  }
}

}  // namespace utils
}  // namespace usb_cam

#endif  // USB_CAM__UTILS_HPP_
