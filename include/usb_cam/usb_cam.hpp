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
}

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "usb_cam/utils.hpp"


namespace usb_cam
{

using utils::io_method;
using utils::pixel_format;
using utils::color_format;

// TODO(lucasw) just store an Image shared_ptr here
// TOOD(flynnev) can new Image shared_ptr be used for both ROS 1 and ROS 2?
//   we should try and eliminate all ROS-specific code from this `usb_cam` lib
//   so we can reuse it for all ROS distros. I think this clock is the only thing
//   left...other than the loggers.
typedef struct
{
  uint32_t width;
  uint32_t height;
  int bytes_per_pixel;
  int image_size;
  struct timespec stamp;
  char * image;
  int is_new;
} camera_image_t;


class UsbCam
{
public:
  UsbCam();
  ~UsbCam();

  // start camera
  bool start(
    const std::string & dev, io_method io, pixel_format pf, color_format cf,
    uint32_t image_width, uint32_t image_height, int framerate);
  // shutdown camera
  bool shutdown(void);

  // grabs a new image from the camera
  // bool get_image(sensor_msgs::msg::Image:::SharedPtr image);
  bool get_image(
    struct timespec & stamp, std::string & encoding,
    uint32_t & height, uint32_t & width, uint32_t & step, std::vector<uint8_t> & data);

  void get_formats();  // std::vector<usb_cam::msg::Format>& formats);

  // enables/disable auto focus
  bool set_auto_focus(int value);

  // Set video device parameters
  bool set_v4l_parameter(const std::string & param, int value);
  bool set_v4l_parameter(const std::string & param, const std::string & value);

  bool stop_capturing(void);
  bool start_capturing(void);
  bool is_capturing();

private:
  int init_decoder(
    int image_width, int image_height, color_format color_format,
    AVCodecID codec_id, const char * codec_name);
  int init_h264_decoder(int image_width, int image_height, color_format cf);
  int init_mjpeg_decoder(int image_width, int image_height, color_format cf);
  bool mjpeg2rgb(char * MJPEG, int len, char * RGB, int NumPixels);
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
  usb_cam::utils::io_method io_;
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
};

}  // namespace usb_cam

#endif  // USB_CAM__USB_CAM_HPP_
