#include <string>
#include <iostream>
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
#include "v4l/device.hpp"
#include "v4l/errors.hpp"

#include "v4l/device/capability.hpp"
#include "v4l/device/video/input.hpp"
#include "v4l/device/video/output.hpp"
#include "v4l/device/video/output.hpp"
#include "v4l/device/audio/input.hpp"
#include "v4l/device/audio/output.hpp"


namespace v4l
{

Device::Device(const std::string & device, int flags)
: v4l::device::Base(resolve_device_path(device).c_str(), flags),
  v4l::device::Capability(),
  v4l::device::video::Input(),
  v4l::device::video::Output()
{
  if (m_fd == -1) {
    auto err_str = device + " [resolved to `" + std::string(m_device) + "`]\n";
        // TOOD: list possible devices;
    throw FailedToOpen(err_str.c_str());
  }

  std::cout << "Opened device: " << get_device_string() << std::endl;

  print_capabilities();
  print_video_input();
  print_video_output();
  print_audio_input();
  print_audio_output();
    // // Get this devices inputs and outputs, if available
    // try {
    //     get_current_video_input();
    //     m_video_input.print();
    //     get_current_video_output();
    // } catch (const v4l::UsbCamError & err) {
    //     // Just print the error and continue on
    //     std::cout << err.what() << std::endl;
    // }

}

} // namespace v4l
