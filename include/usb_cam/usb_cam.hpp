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


#ifndef USB_CAM__USB_CAM_HPP_
#define USB_CAM__USB_CAM_HPP_

extern "C" {
#include <libavcodec/avcodec.h>
#include <linux/videodev2.h>
}

#include <chrono>
#include <memory>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>

#include "usb_cam/utils.hpp"
#include "usb_cam/formats/pixel_format_base.hpp"
#include "usb_cam/formats/av_pixel_format_helper.hpp"

#include "usb_cam/formats/mjpeg.hpp"
#include "usb_cam/formats/mono.hpp"
#include "usb_cam/formats/rgb.hpp"
#include "usb_cam/formats/uyvy.hpp"
#include "usb_cam/formats/yuyv.hpp"
#include "usb_cam/formats/m420.hpp"


namespace usb_cam
{

using usb_cam::utils::io_method_t;
using usb_cam::formats::pixel_format_base;

/// @brief Add more formats here and to driver_supported_formats below as
/// they are added to this library
using usb_cam::formats::RGB8;
using usb_cam::formats::YUYV;
using usb_cam::formats::YUYV2RGB;
using usb_cam::formats::UYVY;
using usb_cam::formats::UYVY2RGB;
using usb_cam::formats::MONO8;
using usb_cam::formats::MONO16;
using usb_cam::formats::Y102MONO8;
using usb_cam::formats::RAW_MJPEG;
using usb_cam::formats::MJPEG2RGB;
using usb_cam::formats::M4202RGB;


/// @brief list all supported formats that this driver supports
std::vector<std::shared_ptr<pixel_format_base>> driver_supported_formats(
  const formats::format_arguments_t & args = formats::format_arguments_t())
{
  std::vector<std::shared_ptr<pixel_format_base>> fmts = {
    std::make_shared<RGB8>(args),
    std::make_shared<YUYV>(args),
    std::make_shared<YUYV2RGB>(args),
    std::make_shared<UYVY>(args),
    std::make_shared<UYVY2RGB>(args),
    std::make_shared<MONO8>(args),
    std::make_shared<MONO16>(args),
    std::make_shared<Y102MONO8>(args),
    std::make_shared<RAW_MJPEG>(args),
    std::make_shared<MJPEG2RGB>(args),
    std::make_shared<M4202RGB>(args),
  };
  return fmts;
}

typedef struct capture_format_t
{
  struct v4l2_fmtdesc format;
  struct v4l2_frmivalenum v4l2_fmt;
} capture_format_t;

typedef struct parameters_t
{
  std::string camera_name;
  std::string device_name;
  std::string frame_id;
  std::string io_method_name;
  std::string camera_info_url;
  std::string pixel_format_name;
  std::string av_device_format;
  int image_width;
  int image_height;
  int framerate;
  int brightness;
  int contrast;
  int saturation;
  int sharpness;
  int gain;
  int white_balance;
  int exposure;
  int focus;
  bool auto_white_balance;
  bool autoexposure;
  bool autofocus;

  parameters_t()
// *INDENT-OFF*
    : camera_name("usb_cam"),
    device_name("/dev/video0"),
    frame_id("camera"),
    io_method_name("mmap"),
    camera_info_url("package://usb_cam/config/camera_info.yaml"),
    pixel_format_name("yuyv2rgb"),
    av_device_format("YUV422P"),
    image_width(600),
    image_height(480),
    framerate(30.0),
    brightness(-1),
    contrast(-1),
    saturation(-1),
    sharpness(-1),
    gain(-1),
    white_balance(-1),
    exposure(-1),
    focus(-1),
    auto_white_balance(true),
    autoexposure(true),
    autofocus(false)
  {
  }
// *INDENT-ON*
} parameters_t;

typedef struct image_t
{
  char * data;
  size_t width;
  size_t height;
  std::shared_ptr<pixel_format_base> pixel_format;
  size_t number_of_pixels;
  size_t bytes_per_line;
  size_t size_in_bytes;
  v4l2_format v4l2_fmt;
  struct timespec stamp;

  size_t set_number_of_pixels()
  {
    number_of_pixels = width * height;
    return number_of_pixels;
  }
  size_t set_bytes_per_line()
  {
    bytes_per_line = width * pixel_format->byte_depth() * pixel_format->channels();
    return bytes_per_line;
  }
  size_t set_size_in_bytes()
  {
    size_in_bytes = height * bytes_per_line;
    return size_in_bytes;
  }

  /// @brief make it a shorter API call to get the pixel format
  unsigned int get_format_fourcc()
  {
    return pixel_format->v4l2();
  }
} image_t;

class UsbCam
{
public:
  UsbCam();
  ~UsbCam();

  /// @brief Configure device, should be called before start
  void configure(parameters_t & parameters, const io_method_t & io_method);

  /// @brief Start the configured device
  void start();

  /// @brief shutdown camera
  void shutdown(void);

  /// @brief Take a new image with device and return it
  ///   To copy the returned image to another format:
  ///   sensor_msgs::msg::Image image_msg;
  ///   auto new_image = get_image();
  ///   image_msg.data.resize(step * height);
  ///   memcpy(&image_msg.data[0], new_image->frame.base, image_msg.data.size());
  char * get_image();

  /// @brief Overload of get_image to allow users to pass
  /// in an image pointer to fill in
  void get_image(char * destination);

  std::vector<capture_format_t> get_supported_formats();

  // enables/disable auto focus
  bool set_auto_focus(int value);

  // Set video device parameters
  bool set_v4l_parameter(const std::string & param, int value);
  bool set_v4l_parameter(const std::string & param, const std::string & value);

  void stop_capturing();
  void start_capturing();

  inline size_t get_image_width()
  {
    return m_image.width;
  }

  inline size_t get_image_height()
  {
    return m_image.height;
  }

  inline size_t get_image_size_in_bytes()
  {
    return m_image.size_in_bytes;
  }

  inline size_t get_image_size_in_pixels()
  {
    return m_image.number_of_pixels;
  }

  inline timespec get_image_timestamp()
  {
    return m_image.stamp;
  }

  /// @brief Get number of bytes per line in image
  /// @return number of bytes per line in image
  inline unsigned int get_image_step()
  {
    return m_image.bytes_per_line;
  }

  inline std::string get_device_name()
  {
    return m_device_name;
  }

  inline std::shared_ptr<pixel_format_base> get_pixel_format()
  {
    return m_image.pixel_format;
  }

  inline usb_cam::utils::io_method_t get_io_method()
  {
    return m_io;
  }

  inline int get_fd()
  {
    return m_fd;
  }

  inline std::shared_ptr<usb_cam::utils::buffer[]> get_buffers()
  {
    return m_buffers;
  }

  inline unsigned int number_of_buffers()
  {
    return m_number_of_buffers;
  }

  inline AVCodec * get_avcodec()
  {
    return m_avcodec;
  }

  inline AVDictionary * get_avoptions()
  {
    return m_avoptions;
  }

  inline AVCodecContext * get_avcodec_context()
  {
    return m_avcodec_context;
  }

  inline AVFrame * get_avframe()
  {
    return m_avframe;
  }

  inline bool is_capturing()
  {
    return m_is_capturing;
  }

  inline time_t get_epoch_time_shift_us()
  {
    return m_epoch_time_shift_us;
  }

  inline std::vector<capture_format_t> supported_formats()
  {
    if (m_supported_formats.size() == 0) {
      this->get_supported_formats();
    }

    return m_supported_formats;
  }

  /// @brief Check if the given format is supported by this device
  /// If it is supported, set the m_pixel_format variable to it.
  /// @param format the format to check if it is supported
  /// @return bool true if the given format is supported, false otherwise
  inline bool set_pixel_format(const formats::format_arguments_t & args)
  {
    bool result = false;

    std::shared_ptr<pixel_format_base> found_driver_format = nullptr;

    // First check if given format is supported by this driver
    for (auto driver_fmt : driver_supported_formats(args)) {
      if (driver_fmt->name() == args.name) {
        found_driver_format = driver_fmt;
      }
    }

    if (found_driver_format == nullptr) {
      // List the supported formats of this driver for the user before throwing
      std::cerr << "This driver supports the following formats:" << std::endl;
      for (auto driver_fmt : driver_supported_formats(args)) {
        std::cerr << "\t" << driver_fmt->name() << std::endl;
      }
      throw std::invalid_argument(
              "Specified format `" + args.name + "` is unsupported by this ROS driver"
      );
    }

    std::cout << "This device supports the following formats:" << std::endl;
    for (auto fmt : this->supported_formats()) {
      // Always list the devices supported formats for the user
      std::cout << "\t" << fmt.format.description << " ";
      std::cout << fmt.v4l2_fmt.width << " x " << fmt.v4l2_fmt.height << " (";
      std::cout << fmt.v4l2_fmt.discrete.denominator / fmt.v4l2_fmt.discrete.numerator << " Hz)";
      std::cout << std::endl;

      if (fmt.v4l2_fmt.pixel_format == found_driver_format->v4l2()) {
        result = true;
        m_image.pixel_format = found_driver_format;
      }
    }

    return result;
  }

  /// @brief Set pixel format from parameter list. Required to have logic within UsbCam object
  /// in case pixel format class requires additional information for conversion function
  /// (e.g. number of pixels, width, height, etc.)
  /// @param parameters list of parameters from which the pixel format is to be set
  /// @return pixel format structure corresponding to a given name
  inline std::shared_ptr<pixel_format_base> set_pixel_format(const parameters_t & parameters)
  {
    // create format arguments structure
    formats::format_arguments_t format_args({
        parameters.pixel_format_name,
        parameters.image_width,
        parameters.image_height,
        m_image.number_of_pixels,
        parameters.av_device_format,
      });

    // Look for specified pixel format
    if (!this->set_pixel_format(format_args)) {
      throw std::invalid_argument(
              "Specified format `" + parameters.pixel_format_name + "` is unsupported by the " +
              "selected device `" + parameters.device_name + "`"
      );
    }

    return m_image.pixel_format;
  }

private:
  void init_read();
  void init_mmap();
  void init_userp();
  void init_device();

  void open_device();
  void grab_image();
  void read_frame();
  void process_image(const char * src, char * & dest, const int & bytes_used);

  void uninit_device();
  void close_device();

  std::string m_device_name;
  usb_cam::utils::io_method_t m_io;
  int m_fd;
  unsigned int m_number_of_buffers;
  std::shared_ptr<usb_cam::utils::buffer[]> m_buffers;
  image_t m_image;

  AVFrame * m_avframe;
  AVCodec * m_avcodec;
  AVDictionary * m_avoptions;
  AVCodecContext * m_avcodec_context;

  bool m_is_capturing;
  int m_framerate;
  const time_t m_epoch_time_shift_us;
  std::vector<capture_format_t> m_supported_formats;
};

}  // namespace usb_cam

#endif  // USB_CAM__USB_CAM_HPP_
