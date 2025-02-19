#ifndef V4L__DEVICE__CAPABILITY_HPP
#define V4L__DEVICE__CAPABILITY_HPP

#include "linux/videodev2.h"

#include "v4l/device/base.hpp"

namespace v4l
{
namespace device
{


/// @brief Extend the v4l2_capability struct with helper methods
class Capability : virtual public v4l::device::Base
{
public:
  Capability();
  Capability(const std::string & device, int flags = O_RDWR | O_NONBLOCK);

  void get_capabilities();

  void print_capabilities();

  bool supports_video_capture();
  bool supports_video_capture_mplane();
  bool supports_video_output();
  bool supports_video_output_mplane();
  bool supports_video_m2m();
  bool supports_video_m2m_mplane();
  bool supports_video_overlay();
  bool supports_vbi_capture();
  bool supports_vbi_output();
  bool supports_sliced_vbi_capture();
  bool supports_sliced_vbi_output();
  bool supports_rds_capture();
  bool supports_video_output_overlay();
  bool supports_hardware_frequency_seeking();
  bool supports_rds_output();
  bool supports_tuner();
  bool supports_audio();
  bool supports_radio();
  bool supports_modulator();
  bool supports_sdr_capture();
  bool supports_ext_pix_format();
  bool supports_sdr_output();
  bool supports_meta_capture();
  bool supports_readwrite();
  bool supports_streaming();
  bool supports_meta_output();
  bool supports_touch();
  bool supports_io_mc();
  bool is_media_controller_centric();

private:
  v4l2_capability m_capabilities;
};


}  // namespace device
}  // namespace v4l

#endif  // V4L__DEVICE_CAPABILITY_HPP
