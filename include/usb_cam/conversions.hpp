// Copyright 2023 Evan Flynn
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


inline void MONO102MONO8(const char * RAW, char * MONO, const int & NumPixels)
{
  int i, j;
  for (i = 0, j = 0; i < (NumPixels << 1); i += 2, j += 1) {
    // first byte is low byte, second byte is high byte; smash together and convert to 8-bit
    MONO[j] = (unsigned char)(((RAW[i + 0] >> 2) & 0x3F) | ((RAW[i + 1] << 6) & 0xC0));
  }
}

inline void YUV4202RGB(char * YUV, char * RGB, const int & width, const int & height)
{
  cv::Size size(height, width);
  cv::Mat cv_img(height, width, CV_8UC1, YUV);
  cv::Mat cv_out(height, width, CV_8UC3, RGB);
  cv::cvtColor(cv_img, cv_out, cv::COLOR_YUV420p2RGB);
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
