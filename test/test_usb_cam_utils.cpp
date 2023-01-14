// Copyright 2022 Evan Flynn
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

#include <gtest/gtest.h>

#include <string>

#include "usb_cam/utils.hpp"


using usb_cam::utils::io_method_from_string;
using usb_cam::utils::pixel_format_from_string;


TEST(test_usb_cam_utils, test_io_method_from_string) {
  usb_cam::utils::io_method_t test_io;

  test_io = io_method_from_string("mmap");
  EXPECT_EQ(test_io, usb_cam::utils::IO_METHOD_MMAP);

  test_io = io_method_from_string("read");
  EXPECT_EQ(test_io, usb_cam::utils::IO_METHOD_READ);

  test_io = io_method_from_string("userptr");
  EXPECT_EQ(test_io, usb_cam::utils::IO_METHOD_USERPTR);

  test_io = io_method_from_string("bananas");
  EXPECT_EQ(test_io, usb_cam::utils::IO_METHOD_UNKNOWN);
}

TEST(test_usb_cam_utils, test_pixel_format_from_string) {
  usb_cam::utils::pixel_format_t test_fmt;

  test_fmt = pixel_format_from_string("yuyv");
  EXPECT_EQ(test_fmt, usb_cam::utils::PIXEL_FORMAT_YUYV);

  test_fmt = pixel_format_from_string("uyvy");
  EXPECT_EQ(test_fmt, usb_cam::utils::PIXEL_FORMAT_UYVY);

  test_fmt = pixel_format_from_string("mjpeg");
  EXPECT_EQ(test_fmt, usb_cam::utils::PIXEL_FORMAT_MJPEG);

  test_fmt = pixel_format_from_string("h264");
  EXPECT_EQ(test_fmt, usb_cam::utils::PIXEL_FORMAT_H264);

  test_fmt = pixel_format_from_string("yuvmono10");
  EXPECT_EQ(test_fmt, usb_cam::utils::PIXEL_FORMAT_YUVMONO10);

  test_fmt = pixel_format_from_string("rgb24");
  EXPECT_EQ(test_fmt, usb_cam::utils::PIXEL_FORMAT_RGB24);

  test_fmt = pixel_format_from_string("grey");
  EXPECT_EQ(test_fmt, usb_cam::utils::PIXEL_FORMAT_GREY);

  test_fmt = pixel_format_from_string("yu12");
  EXPECT_EQ(test_fmt, usb_cam::utils::PIXEL_FORMAT_YU12);

  test_fmt = pixel_format_from_string("apples");
  EXPECT_EQ(test_fmt, usb_cam::utils::PIXEL_FORMAT_UNKNOWN);
}

TEST(test_usb_cam_utils, test_pixel_format_to_string) {
  std::string test_fmt;

  test_fmt = pixel_format_to_string(usb_cam::utils::PIXEL_FORMAT_YUYV);
  EXPECT_EQ(test_fmt, "yuyv");

  test_fmt = pixel_format_to_string(usb_cam::utils::PIXEL_FORMAT_UYVY);
  EXPECT_EQ(test_fmt, "uyvy");

  test_fmt = pixel_format_to_string(usb_cam::utils::PIXEL_FORMAT_MJPEG);
  EXPECT_EQ(test_fmt, "mjpeg");

  test_fmt = pixel_format_to_string(usb_cam::utils::PIXEL_FORMAT_H264);
  EXPECT_EQ(test_fmt, "h264");

  test_fmt = pixel_format_to_string(usb_cam::utils::PIXEL_FORMAT_YUVMONO10);
  EXPECT_EQ(test_fmt, "yuvmono10");

  test_fmt = pixel_format_to_string(usb_cam::utils::PIXEL_FORMAT_RGB24);
  EXPECT_EQ(test_fmt, "rgb24");

  test_fmt = pixel_format_to_string(usb_cam::utils::PIXEL_FORMAT_GREY);
  EXPECT_EQ(test_fmt, "grey");

  test_fmt = pixel_format_to_string(usb_cam::utils::PIXEL_FORMAT_YU12);
  EXPECT_EQ(test_fmt, "yu12");

  test_fmt = pixel_format_to_string(usb_cam::utils::PIXEL_FORMAT_UNKNOWN);
  EXPECT_EQ(test_fmt, "unknown");
}

TEST(test_usb_cam_utils, test_color_format) {
  usb_cam::utils::color_format_t test_fmt;

  test_fmt = usb_cam::utils::color_format_from_string("yuv420p");
  EXPECT_EQ(test_fmt, usb_cam::utils::COLOR_FORMAT_YUV420P);

  test_fmt = usb_cam::utils::color_format_from_string("yuv422p");
  EXPECT_EQ(test_fmt, usb_cam::utils::COLOR_FORMAT_YUV422P);

  test_fmt = usb_cam::utils::color_format_from_string("pears");
  EXPECT_EQ(test_fmt, usb_cam::utils::COLOR_FORMAT_UNKNOWN);
}

TEST(test_usb_cam_utils, test_clip_value) {
  // Clip values to 0 if -128<=val<0
  for (int i = -128; i < 0; i++) {
    EXPECT_EQ(0, usb_cam::utils::CLIPVALUE(i));
  }
  // Do not clip values between 0<val<255
  // Just return the value as unsigned char
  for (int i = 0; i < 255; i++) {
    EXPECT_EQ(i, usb_cam::utils::CLIPVALUE(i));
  }
  // Clip values to 255 if 255<=val<=383
  for (int i = 255; i <= 383; i++) {
    EXPECT_EQ(255, usb_cam::utils::CLIPVALUE(i));
  }
  // Test outlier cases val < -128 and val > 383
  // these will use the old method (non-array method)
  EXPECT_EQ(0, usb_cam::utils::CLIPVALUE(-129));
  EXPECT_EQ(255, usb_cam::utils::CLIPVALUE(400));
}

TEST(test_usb_cam_utils, test_monotonic_to_real_time) {
  // Get timeval to use for test

  const time_t test_time_t = usb_cam::utils::get_epoch_time_shift();

  EXPECT_NE(test_time_t, 0);
}
