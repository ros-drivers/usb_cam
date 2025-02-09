// Copyright 2023 Evan Flynn
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


#ifndef USB_CAM__FORMATS__MJPEG_HPP_
#define USB_CAM__FORMATS__MJPEG_HPP_
///
/// @file Most of this code follows the example provided by ffmpeg:
///
/// https://www.ffmpeg.org/doxygen/5.1/decode__video_8c_source.html
/// https://www.ffmpeg.org/doxygen/4.0/decode__video_8c_source.html
///

#include <iostream>


extern "C" {
#define __STDC_CONSTANT_MACROS  // Required for libavutil
#include "libavutil/imgutils.h"
#include "libavformat/avformat.h"
#include "libavutil/error.h"
#include "libavutil/log.h"
#include "linux/videodev2.h"
#include "libswscale/swscale.h"
}

#include "usb_cam/usb_cam.hpp"
#include "usb_cam/formats/pixel_format_base.hpp"
#include "usb_cam/formats/utils.hpp"
#include "usb_cam/formats/av_pixel_format_helper.hpp"


namespace usb_cam
{
namespace formats
{

class RAW_MJPEG : public pixel_format_base
{
public:
  explicit RAW_MJPEG(const format_arguments_t & args = format_arguments_t())
  : pixel_format_base(
      "raw_mjpeg",
      V4L2_PIX_FMT_MJPEG,
      get_ros_pixel_format_from_av_format(args.av_device_format_str),
      get_channels_from_av_format(args.av_device_format_str),
      get_bit_depth_from_av_format(args.av_device_format_str),
      false)
  {}
};

class MJPEG2RGB : public pixel_format_base
{
public:
  explicit MJPEG2RGB(const format_arguments_t & args = format_arguments_t())
  : pixel_format_base(
      "mjpeg2rgb",
      V4L2_PIX_FMT_MJPEG,
      usb_cam::constants::RGB8,
      3,
      8,
      true),
    m_avcodec(avcodec_find_decoder(AVCodecID::AV_CODEC_ID_MJPEG)),
    m_avparser(av_parser_init(AVCodecID::AV_CODEC_ID_MJPEG)),
    m_avframe_device(av_frame_alloc()),
    m_avframe_rgb(av_frame_alloc()),
    m_avoptions(NULL),
    m_averror_str(reinterpret_cast<char *>(malloc(AV_ERROR_MAX_STRING_SIZE)))
  {
    if (!m_avcodec) {
      throw std::runtime_error("Could not find MJPEG decoder");
    }

    if (!m_avparser) {
      throw std::runtime_error("Could not find MJPEG parser");
    }

    m_avcodec_context = avcodec_alloc_context3(m_avcodec);

    m_avframe_device->width = args.width;
    m_avframe_device->height = args.height;
    m_avframe_device->format = AV_PIX_FMT_YUV422P;
    m_avframe_device->format = formats::get_av_pixel_format_from_string(args.av_device_format_str);

    m_avframe_rgb->width = args.width;
    m_avframe_rgb->height = args.height;
    m_avframe_rgb->format = AV_PIX_FMT_RGB24;

    m_sws_context = sws_getContext(
      args.width, args.height, (AVPixelFormat)m_avframe_device->format,
      args.width, args.height, (AVPixelFormat)m_avframe_rgb->format, SWS_FAST_BILINEAR,
      NULL, NULL, NULL);

    // Suppress warnings from ffmpeg libraries to avoid spamming the console
    av_log_set_level(AV_LOG_FATAL);
    av_log_set_flags(AV_LOG_SKIP_REPEATED);
    // av_log_set_flags(AV_LOG_PRINT_LEVEL);

    m_avcodec_context->width = args.width;
    m_avcodec_context->height = args.height;
    m_avcodec_context->pix_fmt = (AVPixelFormat)m_avframe_device->format;
    m_avcodec_context->codec_type = AVMEDIA_TYPE_VIDEO;

    m_avframe_device_size = static_cast<size_t>(
      av_image_get_buffer_size(
        (AVPixelFormat)m_avframe_device->format,
        m_avframe_device->width,
        m_avframe_device->height,
        m_align));
    m_avframe_rgb_size = static_cast<size_t>(
      av_image_get_buffer_size(
        (AVPixelFormat)m_avframe_rgb->format,
        m_avframe_rgb->width,
        m_avframe_rgb->height,
        m_align));

    // Initialize AVCodecContext
    if (avcodec_open2(m_avcodec_context, m_avcodec, &m_avoptions) < 0) {
      throw std::runtime_error("Could not open decoder");
      return;
    }

    m_result = av_frame_get_buffer(m_avframe_device, m_align);
    if (m_result != 0) {
      print_av_error_string(m_result);
    }
    m_result = av_frame_get_buffer(m_avframe_rgb, m_align);
    if (m_result != 0) {
      print_av_error_string(m_result);
    }
  }

  ~MJPEG2RGB()
  {
    if (m_averror_str) {
      free(m_averror_str);
    }
    if (m_avoptions) {
      free(m_avoptions);
    }
    if (m_avcodec_context) {
      avcodec_free_context(&m_avcodec_context);
    }
    if (m_avframe_device) {
      av_frame_free(&m_avframe_device);
    }
    if (m_avframe_rgb) {
      av_frame_free(&m_avframe_rgb);
    }
    if (m_avparser) {
      av_parser_close(m_avparser);
    }

    if (m_sws_context) {
      sws_freeContext(m_sws_context);
    }
  }

  void convert(const char * & src, char * & dest, const int & bytes_used) override
  {
    m_result = 0;
    // clear the picture
    memset(dest, 0, m_avframe_device_size);

    auto avpacket = av_packet_alloc();
    av_new_packet(avpacket, bytes_used);
    memcpy(avpacket->data, src, bytes_used);

    // Pass src MJPEG image to decoder
    m_result = avcodec_send_packet(m_avcodec_context, avpacket);

    av_packet_free(&avpacket);

    // If result is not 0, report what went wrong
    if (m_result != 0) {
      std::cerr << "Failed to send AVPacket to decode: ";
      print_av_error_string(m_result);
      return;
    }

    m_result = avcodec_receive_frame(m_avcodec_context, m_avframe_device);

    if (m_result == AVERROR(EAGAIN) || m_result == AVERROR_EOF) {
      return;
    } else if (m_result < 0) {
      std::cerr << "Failed to recieve decoded frame from codec: ";
      print_av_error_string(m_result);
      return;
    }

    sws_scale(
      m_sws_context, m_avframe_device->data,
      m_avframe_device->linesize, 0, m_avframe_device->height,
      m_avframe_rgb->data, m_avframe_rgb->linesize);

    av_image_copy_to_buffer(
      const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(dest)),
      m_avframe_rgb_size, m_avframe_rgb->data,
      m_avframe_rgb->linesize, (AVPixelFormat)m_avframe_rgb->format,
      m_avframe_rgb->width, m_avframe_rgb->height, m_align);
  }

private:
  void print_av_error_string(int & err_code)
  {
    av_make_error_string(m_averror_str, AV_ERROR_MAX_STRING_SIZE, err_code);
    std::cerr << m_averror_str << std::endl;
  }

  const AVCodec * m_avcodec;
  AVCodecContext * m_avcodec_context;
  AVCodecParserContext * m_avparser;
  AVFrame * m_avframe_device;
  AVFrame * m_avframe_rgb;
  AVDictionary * m_avoptions;
  SwsContext * m_sws_context;
  size_t m_avframe_device_size;
  size_t m_avframe_rgb_size;
  char * m_averror_str;
  int m_result = 0;

  const int m_align = 32;
};

}  // namespace formats
}  // namespace usb_cam

#endif  // USB_CAM__FORMATS__MJPEG_HPP_
