#ifndef V4L__DEVICE__AUDIO__OUTPUT_HPP
#define V4L__DEVICE__AUDIO__OUTPUT_HPP

#include "linux/videodev2.h"

#include "v4l/device/base.hpp"


namespace v4l
{
namespace device
{
namespace audio
{

/// @brief Extend the v4l2_audioout struct with helper methods
class Output : virtual public v4l::device::Base
{
public:
  void get_audio_output();
  void print_audio_output();

  bool is_audio_output_valid();

  bool supports_stereo();
  bool supports_avl();

  bool is_avl_on();

private:
  bool m_is_valid;
  v4l2_audioout m_output;
};

}  // namespace audio
}  // namespace device
}  // namespace v4l

#endif  // V4L__DEVICE__AUDIO__OUTPUT_HPP
