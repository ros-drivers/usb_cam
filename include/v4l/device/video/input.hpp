#ifndef V4L__DEVICE__VIDEO__INPUT_HPP
#define V4L__DEVICE__VIDEO__INPUT_HPP

#include "linux/videodev2.h"

#include "v4l/device/base.hpp"


namespace v4l
{
namespace device
{
namespace video
{

/// @brief Extend the v4l2_input struct with helper methods
class Input : virtual public v4l::device::Base
{
public:
  Input();
  Input(const std::string & device, int flags = O_RDWR | O_NONBLOCK);

  int get_video_input_index();
  void get_video_input();

  bool is_video_input_valid();

  void print_video_input();

  bool is_tuner();
  bool is_camera();
  bool is_touch();

  bool has_power();
  bool has_signal();
  bool has_color();

  bool is_flipped_horizontally();
  bool is_flipped_vertically();

  bool has_horizontal_sync_lock();
  bool has_vertical_sync_lock();

  bool is_color_killer_active();
  bool has_standard_format_lock();

  bool has_sync_lock();
  bool has_equalizer_lock();
  bool has_carrier();

  bool has_macrovision();
  bool has_access();
  bool has_vtr_time_constant();

  bool supports_dv_timings();
  bool supports_std();
  bool supports_native_size();

protected:
  bool m_is_valid;
  v4l2_input m_video_input;
};

}  // namespace video
}  // namespace device
}  // namespace v4l

#endif  // V4L__DEVICE__VIDEO__INPUT_HPP
