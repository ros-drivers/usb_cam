#include <iostream>

#include "linux/videodev2.h"

#include "v4l/errors.hpp"
#include "v4l/constants.hpp"
#include "v4l/device/capability.hpp"

namespace v4l
{
namespace device
{

using v4l::constants::DIVIDER;

Capability::Capability()
{
  get_capabilities();
}

Capability::Capability(const std::string & device, int flags)
: v4l::device::Base(resolve_device_path(device).c_str(), flags)
{
  get_capabilities();
}

void Capability::get_capabilities()
{
  if (query(VIDIOC_QUERYCAP, &m_capabilities) == -1) {throw NoDeviceCapabilities(m_device);}
}

void Capability::print_capabilities()
{
  std::cout << "cap device: " << get_device_string() << std::endl;

  std::cout << "Driver: " << m_capabilities.driver << std::endl;
  std::cout << "Card: " << m_capabilities.card << std::endl;
  std::cout << "Bus Info: " << m_capabilities.bus_info << std::endl;
  std::cout << "Version: " << (m_capabilities.version >> 16) << ".";
  std::cout << ((m_capabilities.version >> 8) & 0xFF) << ".";
  std::cout << (m_capabilities.version & 0xFF) << std::endl;
  std::cout << DIVIDER << std::endl;
  std::cout << "Capabilities: " << std::endl;

  if (is_media_controller_centric()) {
    std::cout << "  - is meida controller centric" << std::endl;
  } else {
    std::cout << "  - is video node centric" << std::endl;
  }
  if (supports_video_capture()) {
    std::cout << "  - video capture" << std::endl;
  }
  if (supports_video_capture_mplane()) {
    std::cout << "  - video capture mplane" << std::endl;
  }
  if (supports_streaming()) {
    std::cout << "  - streaming I/O" << std::endl;
  }
  if (supports_audio()) {
    std::cout << "  - audio I/O" << std::endl;
  }
  if (supports_tuner()) {
    std::cout << "  - tuner I/O" << std::endl;
  }
  if (supports_ext_pix_format()) {
    std::cout << "  - extended pixel format" << std::endl;
  }
  if (supports_video_overlay()) {
    std::cout << "  - video overlay" << std::endl;
  }
}

bool Capability::supports_video_capture()
{
  return m_capabilities.capabilities & V4L2_CAP_VIDEO_CAPTURE;
}

bool Capability::supports_video_capture_mplane()
{
  return m_capabilities.capabilities & V4L2_CAP_STREAMING;
}

bool Capability::supports_video_output()
{
  return m_capabilities.capabilities & V4L2_CAP_VIDEO_OUTPUT;
}

bool Capability::supports_video_output_mplane()
{
  return m_capabilities.capabilities & V4L2_CAP_VIDEO_OUTPUT_MPLANE;
}

bool Capability::supports_video_m2m()
{
  return m_capabilities.capabilities & V4L2_CAP_VIDEO_M2M;
}

bool Capability::supports_video_m2m_mplane()
{
  return m_capabilities.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE;
}

bool Capability::supports_video_overlay()
{
  return m_capabilities.capabilities & V4L2_CAP_VIDEO_OVERLAY;
}

bool Capability::supports_vbi_capture()
{
  return m_capabilities.capabilities & V4L2_CAP_VBI_CAPTURE;
}

bool Capability::supports_vbi_output()
{
  return m_capabilities.capabilities & V4L2_CAP_VBI_OUTPUT;
}

bool Capability::supports_sliced_vbi_capture()
{
  return m_capabilities.capabilities & V4L2_CAP_SLICED_VBI_CAPTURE;
}

bool Capability::supports_sliced_vbi_output()
{
  return m_capabilities.capabilities & V4L2_CAP_SLICED_VBI_OUTPUT;
}

bool Capability::supports_rds_capture()
{
  return m_capabilities.capabilities & V4L2_CAP_RDS_CAPTURE;
}

bool Capability::supports_video_output_overlay()
{
  return m_capabilities.capabilities & V4L2_CAP_VIDEO_OUTPUT_OVERLAY;
}

bool Capability::supports_hardware_frequency_seeking()
{
  return m_capabilities.capabilities & V4L2_CAP_HW_FREQ_SEEK;
}

bool Capability::supports_rds_output()
{
  return m_capabilities.capabilities & V4L2_CAP_RDS_OUTPUT;
}

bool Capability::supports_tuner()
{
  return m_capabilities.capabilities & V4L2_CAP_TUNER;
}

bool Capability::supports_audio()
{
  return m_capabilities.capabilities & V4L2_CAP_AUDIO;
}

bool Capability::supports_radio()
{
  return m_capabilities.capabilities & V4L2_CAP_RADIO;
}

bool Capability::supports_modulator()
{
  return m_capabilities.capabilities & V4L2_CAP_MODULATOR;
}

bool Capability::supports_sdr_capture()
{
  return m_capabilities.capabilities & V4L2_CAP_SDR_CAPTURE;
}

bool Capability::supports_ext_pix_format()
{
  return m_capabilities.capabilities & V4L2_CAP_EXT_PIX_FORMAT;
}

bool Capability::supports_sdr_output()
{
  return m_capabilities.capabilities & V4L2_CAP_SDR_OUTPUT;
}

bool Capability::supports_meta_capture()
{
  return m_capabilities.capabilities & V4L2_CAP_META_CAPTURE;
}

bool Capability::supports_readwrite()
{
  return m_capabilities.capabilities & V4L2_CAP_READWRITE;
}

bool Capability::supports_streaming()
{
  return m_capabilities.capabilities & V4L2_CAP_STREAMING;
}

bool Capability::supports_meta_output()
{
  return m_capabilities.capabilities & V4L2_CAP_META_OUTPUT;
}

bool Capability::supports_touch()
{
  return m_capabilities.capabilities & V4L2_CAP_TOUCH;
}

bool Capability::supports_io_mc()
{
  return m_capabilities.capabilities & V4L2_CAP_IO_MC;
}

bool Capability::is_media_controller_centric()
{
  return m_capabilities.device_caps & V4L2_CAP_IO_MC;
}

}  // namespace device
}  // namespace usb_cam
