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

extern "C" {
#include <fcntl.h>  // for open()
}

#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>  // for access

#include <cmath>
#include <ctime>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <map>

#include "linux/videodev2.h"

#include "usb_cam/constants.hpp"

namespace fs = std::filesystem;


namespace usb_cam
{
namespace utils
{


/// @brief Read more on IO methods here: https://lwn.net/Articles/240667/
typedef enum
{
  /// @brief read method copies the video frame between user and kernal space
  IO_METHOD_READ,
  /// @brief memory mapped buffers allocated in kernel space
  IO_METHOD_MMAP,
  /// @brief memory buffers allocated in the user space
  IO_METHOD_USERPTR,
  IO_METHOD_UNKNOWN,
} io_method_t;

struct buffer
{
  char * start;
  size_t length;
};


/// @brief Get epoch time shift in microseconds
/// @details Run this at start of process to calculate epoch time shift
/// @ref https://stackoverflow.com/questions/10266451/where-does-v4l2-buffer-timestamp-value-starts-counting
inline time_t get_epoch_time_shift_us()
{
  struct timeval epoch_time;
  struct timespec monotonic_time;

  gettimeofday(&epoch_time, NULL);
  clock_gettime(CLOCK_MONOTONIC, &monotonic_time);

  const int64_t uptime_us =
    monotonic_time.tv_sec * 1000000 + static_cast<int64_t>(
    std::round(monotonic_time.tv_nsec / 1000.0));
  const int64_t epoch_us =
    epoch_time.tv_sec * 1000000 + epoch_time.tv_usec;

  return static_cast<time_t>(epoch_us - uptime_us);
}

/// @brief Calculate image timestamp from buffer time and epoch time shift.
/// In this, the buffer time is first converted into microseconds before the epoch time shift,
/// which is to be given in microseconds is added to it. Afterwards it is split into seconds
/// and nanoseconds for the image timestamp.
inline timespec calc_img_timestamp(const timeval & buffer_time, const time_t & epoch_time_shift_us)
{
  timespec img_timestamp;

  int64_t buffer_time_us = (buffer_time.tv_sec * 1000000) + buffer_time.tv_usec;
  buffer_time_us += epoch_time_shift_us;

  img_timestamp.tv_sec = (buffer_time_us / 1000000);
  img_timestamp.tv_nsec = (buffer_time_us % 1000000) * 1000;

  return img_timestamp;
}

inline int xioctl(int fd, uint64_t request, void * arg)
{
  int r = 0;

  do {
    r = ioctl(fd, request, arg);
    continue;
  } while (-1 == r && EINTR == errno);

  return r;
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

/// \brief List currently available valid V4L2 devices
///   Can be used to check if a device string is valid before
///   starting up.
///
///   Inspired by: http://stackoverflow.com/questions/4290834/how-to-get-a-list-of-video-capture-devices-web-cameras-on-linux-ubuntu-c
inline std::map<std::string, v4l2_capability> available_devices()
{
  // Initialize vector of device strings to fill in
  std::map<std::string, v4l2_capability> v4l2_devices;

  // Get a list of all v4l2 devices from /sys/class/video4linux
  const std::string v4l2_symlinks_dir = "/sys/class/video4linux/";
  for (const auto & device_symlink : fs::directory_iterator(v4l2_symlinks_dir)) {
    if (fs::is_symlink(device_symlink)) {
      // device_str is the full path to the device in /sys/devices/*
      std::string device_str = fs::canonical(
        v4l2_symlinks_dir + fs::read_symlink(
          device_symlink).generic_string());
      // get the proper device name (e.g. /dev/*)
      std::ifstream uevent_file(device_str + "/uevent");
      std::string line;
      std::string device_name;
      while (std::getline(uevent_file, line)) {
        auto dev_name_index = line.find("DEVNAME=");
        if (dev_name_index != std::string::npos) {
          device_name = "/dev/" + line.substr(dev_name_index + 8, line.size());
          break;
        }
      }

      int fd;
      // Try and open device to test access
      if ((fd = open(device_name.c_str(), O_RDONLY)) == -1) {
        std::cerr << "Cannot open device: `" << device_name << "`, ";
        std::cerr << "double-check read / write permissions for device" << std::endl;
      } else {
        struct v4l2_capability device_capabilities = {};
        if (xioctl(fd, VIDIOC_QUERYCAP, &device_capabilities) == -1) {
          std::cerr << "Could not retrieve device capabilities: `" << device_name;
          std::cerr << "`" << std::endl;
        } else {
          v4l2_devices[device_name] = device_capabilities;
        }
      }
    } else {
      // device doesn't exist, continue
      continue;
    }
  }

  return v4l2_devices;
}

}  // namespace utils
}  // namespace usb_cam

#endif  // USB_CAM__UTILS_HPP_
