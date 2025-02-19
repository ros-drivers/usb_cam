#ifndef V4L__DEVICE__AUDIO__INPUT_HPP
#define V4L__DEVICE__AUDIO__INPUT_HPP

#include "linux/videodev2.h"

#include "v4l/device/base.hpp"


namespace v4l
{
namespace device
{
namespace audio
{

/// @brief Extend the v4l2_audio struct with helper methods
class Input : virtual public v4l::device::Base
{
public:
  void get_audio_input();
  void print_audio_input();

  bool is_audio_input_valid();

  bool supports_stereo();
  bool supports_avl();

  bool is_avl_on();

private:
  bool m_is_valid;
  v4l2_audio m_input;
};

}  // namespace audio
}  // namespace device
}  // namespace V4L

#endif  // V4L__DEVICE__AUDIO__INPUT_HPP
