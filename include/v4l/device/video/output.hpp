#ifndef V4L__DEVICE__VIDEO__OUTPUT_HPP
#define V4L__DEVICE__VIDEO__OUTPUT_HPP

#include "linux/videodev2.h"

#include "v4l/device/base.hpp"

namespace v4l
{
namespace device
{
namespace video
{

/// @brief Extend the v4l2_output struct with helper methods
class Output : virtual public v4l::device::Base
{
public:
  int get_video_output_index();
  void get_video_output();
  void print_video_output();

  bool is_video_input_valid();

  bool is_modulator();
  bool is_analog();
  bool is_vga_overlay();

  bool supports_dv_timings();
  bool supports_std();
  bool supports_native_size();

private:
  bool m_is_valid;
  v4l2_output m_output;
};

}  // namespace video
}  // namespace device
}  // namespace v4l

#endif  // V4L__DEVICE__VIDEO__OUTPUT_HPP
