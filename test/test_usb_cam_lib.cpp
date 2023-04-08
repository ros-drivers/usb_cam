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
#include <malloc.h>

#include <chrono>
#include <iostream>
#include <thread>

#include "usb_cam/usb_cam.hpp"
#include "usb_cam/utils.hpp"

#include "sensor_msgs/msg/image.hpp"


namespace
{

class test_usb_cam_lib_fixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    m_test_cam->assign_parameters(m_test_parameters);
    m_test_cam->configure();
    m_test_cam->start();
  }

  void TearDown() override
  {
    m_test_cam->shutdown();
  }

  usb_cam::parameters_t m_test_parameters{
    "test_camera",
    "/dev/video0",
    "test_camera_frame",
    "mmap",
    "package://usb_cam/config/camera_info.yaml",
    "mjpeg2rgb",
    480,
    640,
    30,
    -1,
    -1,
    -1,
    -1,
    -1,
    -1,
    -1,
    -1,
    true,
    true,
    false,
  };

  usb_cam::UsbCam * m_test_cam = new usb_cam::UsbCam();
};

}  // namespace


TEST_F(test_usb_cam_lib_fixture, parametrs) {
  // Test to make sure a few parameters were initilized properly
  // and they can be accessed directly if needed
  ASSERT_EQ(m_test_parameters.camera_name, "test_camera");
  ASSERT_EQ(m_test_parameters.device_name, "/dev/video0");
  ASSERT_EQ(m_test_parameters.frame_id, "test_camera_frame");
}

TEST_F(test_usb_cam_lib_fixture, usb_cam_class_basic) {
  // Ensure parameters were properly set using some helper functions
  ASSERT_EQ(m_test_cam->get_image_width(), size_t(480));
  ASSERT_EQ(m_test_cam->get_image_height(), size_t(640));
  ASSERT_EQ(m_test_cam->get_image_size(), size_t(921600));
  ASSERT_EQ(m_test_cam->get_image_step(), 1440U);
  ASSERT_EQ(m_test_cam->get_device_name(), "/dev/video0");
  ASSERT_EQ(m_test_cam->get_pixel_format()->name(), "mjpeg2rgb");
  ASSERT_EQ(m_test_cam->number_of_buffers(), 4U);
}

TEST_F(test_usb_cam_lib_fixture, usb_cam_class_basic_get_image) {
  auto image = m_test_cam->get_image();

  ASSERT_NE(image, nullptr);

  auto stamp = m_test_cam->get_image_timestamp();

  ASSERT_GT(stamp.tv_sec, 0);
  ASSERT_GT(stamp.tv_nsec, 0);

  ASSERT_EQ(m_test_cam->is_capturing(), true);
  m_test_cam->stop_capturing();
  ASSERT_EQ(m_test_cam->is_capturing(), false);
  m_test_cam->start_capturing();
  ASSERT_EQ(m_test_cam->is_capturing(), true);
}

TEST_F(test_usb_cam_lib_fixture, usb_cam_class_one_copy_get_image) {
  // Pre-allocate image
  char * test_image = reinterpret_cast<char *>(malloc(m_test_cam->get_image_size()));
  // Pass in pointer
  m_test_cam->get_image(test_image);
  ASSERT_NE(test_image, nullptr);

}