// Copyright 2022 Evan Flynn
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
//    * Neither the name of the Evan Flynn nor the names of its
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


#ifndef USB_CAM__CONVERSIONS_HPP_
#define USB_CAM__CONVERSIONS_HPP_

#include <string>

#include "opencv2/imgproc.hpp"

#include "usb_cam/constants.hpp"
#include "usb_cam/utils.hpp"


namespace usb_cam
{
namespace conversions
{


/// @brief Convert a given MJPEG image to RGB
///
/// @param MJPEG Input MJPEG image to convert
/// @param len Length of MJPEG image in memory
/// @param RGB Output RGB image to fill
/// @param NumPixels Length of output RGB image in memory
inline bool MJPEG2RGB(
  usb_cam::UsbCam * usb_cam_obj, char * MJPEG, int len, char * RGB, const int & NumPixels)
{
  int got_picture = 1;

  // clear the picture
  memset(RGB, 0, NumPixels);

  auto avcodec_context_ = usb_cam_obj->get_avcodec_context();
  auto avframe_camera_ = usb_cam_obj->get_avframe_camera();
  auto avframe_camera_size_ = usb_cam_obj->get_avframe_camera_size();
  auto avframe_rgb_ = usb_cam_obj->get_avframe_rgb();
  auto avframe_rgb_size_ = usb_cam_obj->get_avframe_rgb_size();

#if LIBAVCODEC_VERSION_MAJOR > 52
  (void)len;
  (void)MJPEG;
  avcodec_receive_frame(avcodec_context_, avframe_camera_);
  // avcodec_send_packet(avcodec_context_, );
#else
  avcodec_decode_video2(
    avcodec_context_, avframe_camera_, &got_picture, reinterpret_cast<uint8_t *>(MJPEG), len);
#endif

  if (!got_picture) {
    std::cerr << "Webcam: expected picture but didn't get it..." << std::endl;
    return false;
  }

  int xsize = avcodec_context_->width;
  int ysize = avcodec_context_->height;
#if LIBAVCODEC_VERSION_MAJOR > 52
  int pic_size = av_image_get_buffer_size(avcodec_context_->pix_fmt, xsize, ysize, 1);
#else
  // TODO(lucasw) avpicture_get_size corrupts the pix_fmt
  int pic_size = avpicture_get_size(avcodec_context_->pix_fmt, xsize, ysize);
#endif
  // int pic_size = av_image_get_buffer_size(avcodec_context_->pix_fmt, xsize, ysize);
  if (pic_size != avframe_camera_size_) {
    std::cerr << "Outbuf size mismatch.  pic_size: " << pic_size;
    std::cerr << " bufsize: " << avframe_camera_size_ << std::endl;
    return false;
  }
  // TODO(lucasw) why does the image need to be scaled?  Does it also convert formats?
  // RCLCPP_INFO_STREAM(
  //   std::cout << "sw scaler " << xsize << " " << ysize << " "
  //     << avcodec_context_->pix_fmt << ", linesize " << avframe_rgb_->linesize << std::endl;
  // TODO(lucasw) only do if xsize and ysize or pix fmt is different from last time
  usb_cam_obj->get_video_sws();
  usb_cam_obj->scale();
#if LIBAVCODEC_VERSION_MAJOR > 52
  int size = av_image_copy_to_buffer(
    reinterpret_cast<uint8_t *>(avframe_rgb_), pic_size, avframe_camera_->data,
    avframe_camera_->linesize, avcodec_context_->pix_fmt, xsize, ysize, 1);
#else
  int size = avpicture_layout(
    reinterpret_cast<AVPicture *>(avframe_rgb_), AV_PIX_FMT_RGB24,
    xsize, ysize, reinterpret_cast<uint8_t *>(RGB), avframe_rgb_size_);
#endif

  if (size != avframe_rgb_size_) {
    std::cerr << "avpicture_layout error: " << size << std::endl;
    return false;
  }

  return true;
}

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
inline bool YUV2RGB(
  const unsigned char & y, const unsigned char & u, const unsigned char & v,
  unsigned char * r, unsigned char * g, unsigned char * b)
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
  *r = utils::CLIPVALUE(r2);
  *g = utils::CLIPVALUE(g2);
  *b = utils::CLIPVALUE(b2);

  return true;
}

inline bool UYVY2RGB(const char * YUV, char * & RGB, const int & NumPixels)
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

inline bool MONO102MONO8(const char * RAW, char * & MONO, const int & NumPixels)
{
  int i, j;
  for (i = 0, j = 0; i < (NumPixels << 1); i += 2, j += 1) {
    // first byte is low byte, second byte is high byte; smash together and convert to 8-bit
    MONO[j] = (unsigned char)(((RAW[i + 0] >> 2) & 0x3F) | ((RAW[i + 1] << 6) & 0xC0));
  }
  return true;
}

inline bool YUYV2RGB(const char * YUV, char * & RGB, const int & NumPixels)
{
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r, g, b;

  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6) {
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

inline bool COPY2RGB(const char * input, char * & output, const int & NumPixels)
{
  memcpy(output, input, NumPixels * 3);
  return true;
}

inline bool YUV4202RGB(char * YUV, char * & RGB, const int & width, const int & height)
{
  cv::Size size(height, width);
  cv::Mat cv_img(height * 1.5, width, CV_8UC1, YUV);
  cv::Mat cv_out(height, width, CV_8UC3, RGB);

  cvtColor(cv_img, cv_out, cv::COLOR_YUV420p2BGR);
  return true;
}

std::string FCC2S(const unsigned int & val)
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

}  // namespace conversions
}  // namespace usb_cam

#endif  // USB_CAM__CONVERSIONS_HPP_
