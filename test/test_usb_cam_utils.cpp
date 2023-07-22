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

#include <gtest/gtest.h>
#include <libavutil/pixfmt.h>

#include <string>

#include "usb_cam/utils.hpp"
#include "usb_cam/formats/utils.hpp"


using usb_cam::utils::io_method_from_string;


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

TEST(test_usb_cam_utils, test_clip_value) {
  // Clip values to 0 if -128<=val<0
  for (int i = -128; i < 0; i++) {
    EXPECT_EQ(0, usb_cam::formats::CLIPVALUE(i));
  }
  // Do not clip values between 0<val<255
  // Just return the value as unsigned char
  for (int i = 0; i < 255; i++) {
    EXPECT_EQ(i, usb_cam::formats::CLIPVALUE(i));
  }
  // Clip values to 255 if 255<=val<=383
  for (int i = 255; i <= 383; i++) {
    EXPECT_EQ(255, usb_cam::formats::CLIPVALUE(i));
  }
  // Test outlier cases val < -128 and val > 383
  // these will use the old method (non-array method)
  EXPECT_EQ(0, usb_cam::formats::CLIPVALUE(-129));
  EXPECT_EQ(255, usb_cam::formats::CLIPVALUE(400));
}

TEST(test_usb_cam_utils, test_monotonic_to_real_time) {
  // Get timeval to use for test

  const time_t test_time_t = usb_cam::utils::get_epoch_time_shift_us();

  EXPECT_NE(test_time_t, 0);
}
