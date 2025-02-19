#include <iostream>
#include "linux/videodev2.h"

#include "v4l/device/video/output.hpp"
#include "v4l/errors.hpp"


namespace v4l
{
namespace device
{
namespace video
{

void Output::print_video_output()
{
  std::cout << DIVIDER << std::endl;
  if (!is_video_input_valid()) {
    std::cout << "No valid video output available" << std::endl;
    return;
  }
  std::cout << "output device: " << get_device_string() << std::endl;
  std::cout << "Current video output:" << std::endl;
  std::cout << "  Name: " << m_output.name << std::endl;
  std::cout << "  Index: " << m_output.index << std::endl;
  std::cout << "  Type:  ";
  if (is_modulator()) {std::cout << "modulator" << std::endl;}
  if (is_analog()) {std::cout << "analog" << std::endl;}
  if (is_vga_overlay()) {std::cout << "vga overlay" << std::endl;}

}

int Output::get_video_output_index()
{
  int output_index;
  if (query(VIDIOC_G_OUTPUT, &output_index) == -1) {
    output_index = -1;
  }
  return output_index;
}

void Output::get_video_output()
{
  memset(&m_output, 0, sizeof(m_output));
  int output_index = get_video_output_index();
     // No video output available, return early
  if (output_index == -1) {
    m_is_valid = false;
    return;
  }

  m_is_valid = true;
  m_output.index = output_index;

  if (query(VIDIOC_ENUMOUTPUT, &m_output) == -1) {
    auto err_str = "Could not retrieve device output: " + std::string(m_device);
    throw VideoOutputError(err_str);
  }
}

bool Output::is_video_input_valid()
{
  return m_is_valid;
}


bool Output::is_modulator()
{
  return m_output.type == V4L2_OUTPUT_TYPE_MODULATOR;
}

bool Output::is_analog()
{
  return m_output.type == V4L2_OUTPUT_TYPE_ANALOG;
}

bool Output::is_vga_overlay()
{
  return m_output.type == V4L2_OUTPUT_TYPE_ANALOGVGAOVERLAY;
}

bool Output::supports_dv_timings()
{
  return m_output.capabilities & V4L2_OUT_CAP_DV_TIMINGS;
}

bool Output::supports_std()
{
  return m_output.capabilities & V4L2_OUT_CAP_STD;
}

bool Output::supports_native_size()
{
  return m_output.capabilities & V4L2_OUT_CAP_NATIVE_SIZE;
}

}  // namespace video
}  // namespace device
}  // namespace v4l
