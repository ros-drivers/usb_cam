// Copyright 2026 sungwanha
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
//    * Neither the name of the copyright holder nor the names of its
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

#include <algorithm>
#include <array>
#include <memory>

#include "usb_cam/usb_cam.hpp"

TEST(test_image_payload, raw_mjpeg_copies_only_current_payload)
{
  std::array<char, 32> source;
  std::array<char, 32> destination;
  usb_cam::image_t image{};
  usb_cam::formats::format_arguments_t args{};
  args.av_device_format_str = "YUV422P";
  image.pixel_format = std::make_shared<usb_cam::formats::RAW_MJPEG>(args);
  image.data = destination.data();
  image.size_in_bytes = destination.size();

  // A shorter frame must not copy padding or bytes left over from a previous frame.
  for (const int length : {9, 3, 12, 0, 32}) {
    source.fill('p');
    std::fill_n(source.begin(), length, 'j');
    destination.fill('x');
    image.process_image(source.data(), length);
    EXPECT_EQ(image.data_size, static_cast<size_t>(length));
    EXPECT_EQ(image.size_in_bytes, 32U);
    EXPECT_TRUE(std::all_of(destination.begin(), destination.begin() + length,
      [](char byte) {return byte == 'j';}));
    EXPECT_TRUE(std::all_of(destination.begin() + length, destination.end(),
      [](char byte) {return byte == 'x';}));
  }
}

TEST(test_image_payload, rejects_invalid_mjpeg_lengths_without_copying)
{
  std::array<char, 32> source{};
  std::array<char, 32> destination;
  usb_cam::image_t image{};
  usb_cam::formats::format_arguments_t args{};
  args.av_device_format_str = "YUV422P";
  image.pixel_format = std::make_shared<usb_cam::formats::RAW_MJPEG>(args);
  image.data = destination.data();
  image.size_in_bytes = destination.size();
  for (const int length : {-1, 33}) {
    destination.fill('x');
    image.data_size = 9;
    EXPECT_THROW(image.process_image(source.data(), length), std::invalid_argument);
    EXPECT_EQ(image.data_size, 0U);
    EXPECT_TRUE(std::all_of(destination.begin(), destination.end(),
      [](char byte) {return byte == 'x';}));
  }
}

TEST(test_image_payload, unconverted_yuyv_keeps_full_output_size)
{
  std::array<char, 8> source = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'};
  std::array<char, 8> destination{};
  usb_cam::image_t image{};
  image.pixel_format = std::make_shared<usb_cam::formats::YUYV>();
  image.data = destination.data();
  image.size_in_bytes = destination.size();
  image.process_image(source.data(), source.size());
  EXPECT_EQ(image.data_size, 8U);
  EXPECT_EQ(destination, source);
}

TEST(test_image_payload, converted_yuyv_uses_output_size_not_input_size)
{
  std::array<unsigned char, 4> source = {0, 128, 0, 128};
  std::array<char, 7> destination;
  destination.fill('x');
  usb_cam::image_t image{};
  usb_cam::formats::format_arguments_t args{};
  args.pixels = 2;
  image.pixel_format = std::make_shared<usb_cam::formats::YUYV2RGB>(args);
  image.data = destination.data();
  image.size_in_bytes = 6;
  image.process_image(reinterpret_cast<const char *>(source.data()), source.size());
  EXPECT_EQ(image.data_size, 6U);
  EXPECT_EQ(destination[6], 'x');
  EXPECT_TRUE(std::all_of(destination.begin(), destination.begin() + 6,
    [](char byte) {return byte == 0;}));
}
