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


#ifndef USB_CAM__FORMATS__MONO_HPP_
#define USB_CAM__FORMATS__MONO_HPP_

#include "linux/videodev2.h"

#include "usb_cam/formats/pixel_format_base.hpp"
#include "usb_cam/formats/utils.hpp"


namespace usb_cam
{
namespace formats
{

class MONO8 : public pixel_format_base
{
public:
  explicit MONO8(const format_arguments_t & args = format_arguments_t())
  : pixel_format_base(
      "mono8",
      V4L2_PIX_FMT_GREY,
      usb_cam::constants::MONO8,
      1,
      8,
      false)
  {
    (void)args;
  }
};


class MONO16 : public pixel_format_base
{
public:
  explicit MONO16(const format_arguments_t & args = format_arguments_t())
  : pixel_format_base(
      "mono16",
      V4L2_PIX_FMT_Y16,
      usb_cam::constants::MONO16,
      1,
      16,
      false)
  {
    (void)args;
  }
};


/// @brief Also known as MONO10 to MONO8
class Y102MONO8 : public pixel_format_base
{
public:
  explicit Y102MONO8(const format_arguments_t & args = format_arguments_t())
  : pixel_format_base(
      "y102mono8",
      V4L2_PIX_FMT_Y10,
      usb_cam::constants::MONO8,
      1,
      8,
      true),
    m_number_of_pixels(args.pixels)
  {}

  /// @brief Convert a Y10 (MONO10) image to MONO8
  /// @param src pointer to source Y10 (MONO10) image
  /// @param dest pointer to destination MONO8 image
  /// @param bytes_used number of bytes used by source image
  void convert(const char * & src, char * & dest, const int & bytes_used) override
  {
    (void)bytes_used;  // not used by this conversion method
    auto src_ = reinterpret_cast<const uint64_t *>(src);
    auto dest_ = reinterpret_cast<uint32_t *>(dest);
    for (int i = 0; i < m_number_of_pixels / 4; i++) {
      dest_[i] = (uint32_t)((src_[i] >> 2)  & 0x000000FF) |
                 (uint32_t)((src_[i] >> 10) & 0x0000FF00) |
                 (uint32_t)((src_[i] >> 18) & 0x00FF0000) |
                 (uint32_t)((src_[i] >> 26) & 0xFF000000);
    }
  }

private:
  int m_number_of_pixels;
};


class Y102MONO16 : public pixel_format_base
{
public:
  explicit Y102MONO16(const format_arguments_t & args = format_arguments_t())
  : pixel_format_base(
      "y102mono16",
      V4L2_PIX_FMT_Y10,
      usb_cam::constants::MONO16,
      1,
      16,
      true), 
    m_number_of_pixels(args.pixels)
  {}

  void convert(const char * & src, char * & dest, const int & bytes_used) override
  {
    auto src_ = reinterpret_cast<const uint64_t *>(src);
    auto dest_ = reinterpret_cast<uint64_t *>(dest);
    (void)bytes_used;  // not used by this conversion method
    for (int i = 0; i < m_number_of_pixels / 4; i++) {
      dest_[i] = src_[i] << 6;
    }
  }

  private:
    int m_number_of_pixels;
};

}  // namespace formats
}  // namespace usb_cam

#endif  // USB_CAM__FORMATS__MONO_HPP_
