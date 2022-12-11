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
#include <libswscale/swscale.h>
#include <linux/videodev2.h>
}

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "usb_cam/utils.hpp"


namespace usb_cam
{

using utils::io_method_t;
using utils::pixel_format_t;
using utils::color_format_t;

// TODO(lucasw) just store an Image shared_ptr here
// TOOD(flynnev) can new Image shared_ptr be used for both ROS 1 and ROS 2?
//   we should try and eliminate all ROS-specific code from this `usb_cam` lib
//   so we can reuse it for all ROS distros. I think this clock is the only thing
//   left...other than the loggers.
typedef struct
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
} camera_image_t;


typedef struct
{
  struct v4l2_fmtdesc format;
  struct v4l2_frmsizeenum size;
  struct v4l2_frmivalenum interval;
} capture_format_t;


class UsbCam
{
public:
  UsbCam();
  ~UsbCam();

  // start camera
  bool start(
    const std::string & dev,
    io_method_t io_method, pixel_format_t pixel_format, color_format_t color_format,
    uint32_t image_width, uint32_t image_height, int framerate);

  // shutdown camera
  bool shutdown(void);

  /// @brief Take a new image with device and return it
  ///   To copy the returned image to another format:
  ///   sensor_msgs::msg::Image image_msg;
  ///   auto new_image = get_image();
  ///   image_msg.data.resize(step * height);
  ///   memcpy(&image_msg.data[0], new_image->image, image_msg.data.size());
  camera_image_t * get_image();

  std::vector<capture_format_t> get_supported_formats();

  // enables/disable auto focus
  bool set_auto_focus(int value);

  // Set video device parameters
  bool set_v4l_parameter(const std::string & param, int value);
  bool set_v4l_parameter(const std::string & param, const std::string & value);

  bool stop_capturing(void);
  bool start_capturing(void);

  inline void scale()
  {
    sws_scale(
      video_sws_, avframe_camera_->data, avframe_camera_->linesize,
      0, avcodec_context_->height, avframe_rgb_->data, avframe_rgb_->linesize);
    // TODO(lucasw) keep around until parameters change
    sws_freeContext(video_sws_);
  }

  inline std::string get_camera_dev()
  {
    return camera_dev_;
  }

  inline unsigned int get_pixelformat()
  {
    return pixelformat_;
  }

  inline bool is_monochrome()
  {
    return monochrome_;
  }

  inline usb_cam::utils::io_method_t get_io_method()
  {
    return io_;
  }

  inline int get_fd()
  {
    return fd_;
  }

  inline usb_cam::utils::buffer * get_buffers()
  {
    return buffers_;
  }

  inline unsigned int number_of_buffers()
  {
    return n_buffers_;
  }

  inline AVFrame * get_avframe_camera()
  {
    return avframe_camera_;
  }

  inline AVFrame * get_avframe_rgb()
  {
    return avframe_rgb_;
  }

  inline AVCodec * get_avcodec()
  {
    return avcodec_;
  }

  inline AVDictionary * get_avoptions()
  {
    return avoptions_;
  }

  inline AVCodecContext * get_avcodec_context()
  {
    return avcodec_context_;
  }

  inline int get_avframe_camera_size()
  {
    return avframe_camera_size_;
  }

  inline int get_avframe_rgb_size()
  {
    return avframe_rgb_size_;
  }

  inline struct SwsContext * get_video_sws()
  {
    video_sws_ = sws_getContext(
      avcodec_context_->width, avcodec_context_->height, avcodec_context_->pix_fmt,
      avcodec_context_->width, avcodec_context_->height,
      AV_PIX_FMT_RGB24, SWS_BILINEAR, NULL, NULL, NULL);
    return video_sws_;
  }

  inline camera_image_t * get_current_image()
  {
    return image_;
  }

  inline bool is_capturing()
  {
    return is_capturing_;
  }

  inline time_t get_epoch_time_shift()
  {
    return epoch_time_shift_;
  }

  inline std::vector<capture_format_t> supported_formats()
  {
    return supported_formats_;
  }

private:
  int init_decoder(
    int image_width, int image_height, color_format_t color_format,
    AVCodecID codec_id, const char * codec_name);
  int init_h264_decoder(int image_width, int image_height, color_format_t cf);
  int init_mjpeg_decoder(int image_width, int image_height, color_format_t cf);
  bool process_image(const void * src, int len, camera_image_t * dest);
  bool read_frame();
  bool uninit_device(void);
  bool init_read(unsigned int buffer_size);
  bool init_mmap(void);
  bool init_userp(unsigned int buffer_size);
  bool init_device(uint32_t image_width, uint32_t image_height, int framerate);
  bool close_device(void);
  bool open_device(void);
  bool grab_image();

  std::string camera_dev_;
  unsigned int pixelformat_;
  bool monochrome_;
  usb_cam::utils::io_method_t io_;
  int fd_;
  usb_cam::utils::buffer * buffers_;
  unsigned int n_buffers_;
  AVFrame * avframe_camera_;
  AVFrame * avframe_rgb_;
  AVCodec * avcodec_;
  AVDictionary * avoptions_;
  AVCodecContext * avcodec_context_;
  int avframe_camera_size_;
  int avframe_rgb_size_;
  struct SwsContext * video_sws_;
  camera_image_t * image_;
  bool is_capturing_;
  const time_t epoch_time_shift_;
  std::vector<capture_format_t> supported_formats_;
};

}  // namespace usb_cam

#endif  // USB_CAM__USB_CAM_HPP_
