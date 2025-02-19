#include <iostream>
#include "linux/videodev2.h"

#include "v4l/errors.hpp"
#include "v4l/device/video/input.hpp"


namespace v4l
{
namespace device
{
namespace video
{

Input::Input()
{
  get_video_input();
}

Input::Input(const std::string & device, int flags)
: v4l::device::Base(resolve_device_path(device).c_str(), flags)
{
  get_video_input();
}

int Input::get_video_input_index()
{
  int input_index;
  if (query(VIDIOC_G_INPUT, &input_index) == -1) {
    input_index = -1;
  }
  return input_index;
}

void Input::get_video_input()
{
  memset(&m_video_input, 0, sizeof(m_video_input));
  int input_index = get_video_input_index();
     // No video input available, return early
  if (input_index == -1) {
    m_is_valid = false;
    return;
  }

  m_is_valid = true;
  m_video_input.index = input_index;
  if (query(VIDIOC_ENUMINPUT, &m_video_input) == -1) {
    auto err_str = "Could not retrieve device input: " + input_index;
    throw VideoInputError(err_str);
  }
}

bool Input::is_video_input_valid()
{
  return m_is_valid;
}

void Input::print_video_input()
{
  std::cout << DIVIDER << std::endl;
  if (!is_video_input_valid()) {
    std::cout << "No input video device available" << std::endl;
    return;
  }
  std::cout << "input device: " << get_device_string() << std::endl;
  std::cout << "Current video input:" << std::endl;
  std::cout << "  Name: " << m_video_input.name << std::endl;
  std::cout << "  Index: " << m_video_input.index << std::endl;
  std::cout << "  Type:  ";
  if (is_tuner()) {std::cout << "tuner" << std::endl;}
  if (is_camera()) {std::cout << "camera" << std::endl;}
  if (is_touch()) {std::cout << "touch" << std::endl;}
  std::cout << "  Status: " << std::endl;
  std::cout << "    General:" << std::endl;
  std::cout << "      - power: ";
  if (!has_power()) {std::cout << "off" << std::endl;} else {std::cout << "on" << std::endl;}
  std::cout << "      - signal: ";
  if (has_signal()) {std::cout << "on" << std::endl;} else {std::cout << "off" << std::endl;}
  std::cout << "      - color: ";
  if (has_color()) {std::cout << "on" << std::endl;} else {std::cout << "off" << std::endl;}

  std::cout << "    Sensor orientation:" << std::endl;
  std::cout << "      - flipped horizontally: ";
  if (is_flipped_horizontally()) {std::cout << "yes" << std::endl;} else {
    std::cout << "no" << std::endl;
  }
  std::cout << "      - flipped vertically: ";
  if (is_flipped_vertically()) {std::cout << "yes" << std::endl;} else {
    std::cout << "no" << std::endl;
  }

  std::cout << "    Analog:" << std::endl;
  std::cout << "      - horizontal sync lock: ";
  if (has_horizontal_sync_lock()) {std::cout << "yes" << std::endl;} else {
    std::cout << "no" << std::endl;
  }
  std::cout << "      - vertical sync lock: ";
  if (has_vertical_sync_lock()) {std::cout << "yes" << std::endl;} else {
    std::cout << "no" << std::endl;
  }
  std::cout << "      - color killer active: ";
  if (is_color_killer_active()) {std::cout << "yes" << std::endl;} else {
    std::cout << "no" << std::endl;
  }
  std::cout << "      - standard format lock: ";
  if (has_standard_format_lock()) {std::cout << "yes" << std::endl;} else {
    std::cout << "no" << std::endl;
  }

  std::cout << "    Digital:" << std::endl;
  std::cout << "      - sync lock: ";
  if (has_sync_lock()) {std::cout << "yes" << std::endl;} else {std::cout << "no" << std::endl;}
  std::cout << "      - equalizer lock: ";
  if (has_equalizer_lock()) {std::cout << "yes" << std::endl;} else {
    std::cout << "no" << std::endl;
  }
  std::cout << "      - carrier recovery: ";
  if (has_carrier()) {std::cout << "yes" << std::endl;} else {std::cout << "no" << std::endl;}

  std::cout << "    VCR and set-top box:" << std::endl;
  std::cout << "      - macrovision: ";
  if (has_macrovision()) {std::cout << "yes" << std::endl;} else {std::cout << "no" << std::endl;}
  std::cout << "      - conditional access: ";
  if (has_access()) {std::cout << "yes" << std::endl;} else {std::cout << "no" << std::endl;}
  std::cout << "      - VTR time constant: ";
  if (has_vtr_time_constant()) {std::cout << "yes" << std::endl;} else {
    std::cout << "no" << std::endl;
  }

  std::cout << "    Capabilities:" << std::endl;
  std::cout << "      - supports S_DV_TIMINGS: ";
  if (supports_dv_timings()) {std::cout << "yes" << std::endl;} else {
    std::cout << "no" << std::endl;
  }
  std::cout << "      - supports S_STD: ";
  if (supports_std()) {std::cout << "yes" << std::endl;} else {std::cout << "no" << std::endl;}
  std::cout << "      - supports setting native size: ";
  if (supports_native_size()) {std::cout << "yes" << std::endl;} else {
    std::cout << "no" << std::endl;
  }

  std::cout << "  Audioset: " << m_video_input.audioset << std::endl;
  std::cout << "  Tuner: " << m_video_input.tuner << std::endl;
  std::cout << "  Standard ID: " << m_video_input.std << std::endl;
}

bool Input::is_tuner()
{
  return m_video_input.type == V4L2_INPUT_TYPE_TUNER;

}
bool Input::is_camera()
{
  return m_video_input.type == V4L2_INPUT_TYPE_CAMERA;
}
bool Input::is_touch()
{
  return m_video_input.type == V4L2_INPUT_TYPE_TOUCH;
}

bool Input::has_power()
{
  return !(m_video_input.status & V4L2_IN_ST_NO_POWER);
}
bool Input::has_signal()
{
  return !(m_video_input.status & V4L2_IN_ST_NO_SIGNAL);
}
bool Input::has_color()
{
  return !(m_video_input.status & V4L2_IN_ST_NO_COLOR);
}

bool Input::is_flipped_horizontally()
{
  return m_video_input.status & V4L2_IN_ST_HFLIP;
}

bool Input::is_flipped_vertically()
{
  return m_video_input.status & V4L2_IN_ST_VFLIP;
}

bool Input::has_horizontal_sync_lock()
{
  return !(m_video_input.status & V4L2_IN_ST_NO_H_LOCK);
}
bool Input::has_vertical_sync_lock()
{
  return !(m_video_input.status & V4L2_IN_ST_NO_V_LOCK);
}

bool Input::is_color_killer_active()
{
  return m_video_input.status & V4L2_IN_ST_COLOR_KILL;
}
bool Input::has_standard_format_lock()
{
  return !(m_video_input.status & V4L2_IN_ST_NO_STD_LOCK);
}

bool Input::has_sync_lock()
{
  return !(m_video_input.status & V4L2_IN_ST_NO_SYNC);
}
bool Input::has_equalizer_lock()
{
  return !(m_video_input.status & V4L2_IN_ST_NO_EQU);
}
bool Input::has_carrier()
{
  return !(m_video_input.status & V4L2_IN_ST_NO_CARRIER);
}

bool Input::has_macrovision()
{
  return m_video_input.status & V4L2_IN_ST_MACROVISION;
}
bool Input::has_access()
{
  return !(m_video_input.status & V4L2_IN_ST_NO_ACCESS);
}
bool Input::has_vtr_time_constant()
{
  return m_video_input.status & V4L2_IN_ST_VTR;
}

bool Input::supports_dv_timings()
{
  return m_video_input.status & V4L2_IN_CAP_DV_TIMINGS;
}
bool Input::supports_std()
{
  return m_video_input.status & V4L2_IN_CAP_STD;
}
bool Input::supports_native_size()
{
  return m_video_input.status & V4L2_IN_CAP_NATIVE_SIZE;
}

}  // namespace video
}  // namespace device
}  // namespace v4l
