#ifndef V4L__DEVICE_HPP
#define V4L__DEVICE_HPP
#include <string>
#include <stdexcept>
#include <filesystem>
#include <algorithm>

#include "linux/videodev2.h"
#include <cstdint>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdexcept>

#include "v4l/constants.hpp"
#include "v4l/device/base.hpp"
#include "v4l/device/capability.hpp"
#include "v4l/device/video/input.hpp"
#include "v4l/device/video/output.hpp"
#include "v4l/device/audio/input.hpp"
#include "v4l/device/audio/output.hpp"


namespace v4l
{

using constants::DIVIDER;

/// @brief A RAII class for interacting with a V4L2 device
class Device
  : virtual public v4l::device::Base,
  public v4l::device::Capability,
  public v4l::device::video::Input,
  public v4l::device::video::Output,
  public v4l::device::audio::Input,
  public v4l::device::audio::Output
{
public:
  Device(const std::string & device, int flags = O_RDWR | O_NONBLOCK);
};

} // namespace v4l

#endif // V4L__DEVICE_HPP
