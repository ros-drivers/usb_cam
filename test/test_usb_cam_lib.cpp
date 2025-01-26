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

#include <chrono>
#include <iostream>
#include <thread>

#include "usb_cam/usb_cam.hpp"
#include "usb_cam/utils.hpp"

TEST(test_usb_cam_lib, test_usb_cam_class) {
  usb_cam::UsbCam test_usb_cam;

  usb_cam::parameters_t parameters;
  test_usb_cam.configure(parameters, usb_cam::utils::IO_METHOD_MMAP);

  test_usb_cam.start();

  auto supported_fmts = test_usb_cam.get_supported_formats();

  // TODO(flynneva): iterate over availble formats with test_usb_cam obj
  for (auto fmt : supported_fmts) {
    std::cerr << "format: " << fmt.format.type << std::endl;
  }

  // TODO(flynneva): uncomment once /dev/video0 can be simulated in CI
  EXPECT_TRUE(test_usb_cam.is_capturing());
  test_usb_cam.shutdown();
}
