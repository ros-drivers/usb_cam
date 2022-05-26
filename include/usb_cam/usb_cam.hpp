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
#include "usb_cam/usb_cam_utils.hpp"

#include <asm/types.h>          /* for videodev2.h */

extern "C"
{
#include <linux/videodev2.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/mem.h>
}

// legacy reasons
#include <libavcodec/version.h>
#if LIBAVCODEC_VERSION_MAJOR < 55
#define AV_CODEC_ID_MJPEG CODEC_ID_MJPEG
#endif

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
// #include <sensor_msgs/msg/image.h>
#include <string>
#include <vector>
#include <sstream>

namespace usb_cam
{


class UsbCam
{
public:
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
    PIXEL_FORMAT_H264,
    PIXEL_FORMAT_UNKNOWN
  } pixel_format;

  typedef enum
  {
    COLOR_FORMAT_YUV420P, 
    COLOR_FORMAT_YUV422P, 
    COLOR_FORMAT_UNKNOWN,
  } color_format;

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
    builtin_interfaces::msg::Time & stamp, std::string & encoding,
    uint32_t & height, uint32_t & width, uint32_t & step, std::vector<uint8_t> & data);

  void get_formats();  // std::vector<usb_cam::msg::Format>& formats);

  // enables/disable auto focus
  bool set_auto_focus(int value);

  // Set video device parameters
  bool set_v4l_parameter(const std::string & param, int value);
  bool set_v4l_parameter(const std::string & param, const std::string & value);

  static io_method io_method_from_string(const std::string & str);
  static pixel_format pixel_format_from_string(const std::string & str);
  static color_format color_format_from_string(const std::string& str);

  bool stop_capturing(void);
  bool start_capturing(void);
  bool is_capturing();

private:
  // TODO(lucasw) just store an Image shared_ptr here
  typedef struct
  {
    uint32_t width;
    uint32_t height;
    int bytes_per_pixel;
    int image_size;
    builtin_interfaces::msg::Time stamp;
    char * image;
    int is_new;
  } camera_image_t;

  struct buffer
  {
    void * start;
    size_t length;
  };


  int init_decoder(int image_width, int image_height, color_format color_format, 
    AVCodecID codec_id, const char *codec_name);
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

  rclcpp::Clock::SharedPtr clock_;
  std::string camera_dev_;
  unsigned int pixelformat_;
  bool monochrome_;
  io_method io_;
  int fd_;
  buffer * buffers_;
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
};

}  // namespace usb_cam

#endif  // USB_CAM__USB_CAM_HPP_
