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


#ifndef USB_CAM__FORMATS__M420_HPP_
#define USB_CAM__FORMATS__M420_HPP_

#include "linux/videodev2.h"

#include "opencv2/imgproc.hpp"

#include "usb_cam/formats/pixel_format_base.hpp"
#include "usb_cam/formats/utils.hpp"


namespace usb_cam
{
namespace formats
{

class M4202RGB : public pixel_format_base
{
public:
  explicit M4202RGB(const format_arguments_t & args = format_arguments_t())
  : pixel_format_base(
      "m4202rgb",
      V4L2_PIX_FMT_M420,
      usb_cam::constants::RGB8,
      3,
      8,
      true),
    m_width(args.width),
    m_height(args.height)
  {}

  /// @brief Convert a YUV420 (aka M420) image to RGB8
  void convert(const char * & src, char * & dest, const int & bytes_used) override
  {
    (void)bytes_used;    // not used by this conversion method
    cv::Size size(m_height, m_width);
    const cv::Mat cv_img(m_height, m_width, CV_8UC1, const_cast<char *>(src));
    cv::Mat cv_out(m_height, m_width, CV_8UC3, dest);
    cv::cvtColor(cv_img, cv_out, cv::COLOR_YUV420p2RGB);
  }

private:
  int m_width;
  int m_height;
};

}  // namespace formats
}  // namespace usb_cam

#endif  // USB_CAM__FORMATS__M420_HPP_
