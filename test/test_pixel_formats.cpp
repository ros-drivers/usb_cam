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

#include <linux/videodev2.h>

#include "usb_cam/formats/pixel_format_base.hpp"

TEST(test_pixel_formats, pixel_format_base_class) {
  auto test_pix_fmt = usb_cam::formats::default_pixel_format();

  EXPECT_EQ(test_pix_fmt.name(), "yuyv");
  EXPECT_EQ(test_pix_fmt.v4l2(), V4L2_PIX_FMT_YUYV);
  EXPECT_EQ(test_pix_fmt.channels(), 2);
  EXPECT_EQ(test_pix_fmt.bit_depth(), 8);
  EXPECT_EQ(test_pix_fmt.requires_conversion(), false);

  EXPECT_EQ(test_pix_fmt.is_bayer(), false);
  // TOOD(flynneva): should this be true for `yuyv`?
  EXPECT_EQ(test_pix_fmt.is_color(), false);
  EXPECT_EQ(test_pix_fmt.is_mono(), false);
}
