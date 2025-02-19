#ifndef V4L__BASE_HPP_
#define V4L__BASE_HPP_
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


namespace v4l
{
namespace device
{

using ::v4l::constants::DIVIDER;

/// @brief A RAII class for interacting with a V4L2 device
class Base
{
public:
  Base();
  Base(const std::string & device, int flags = O_RDWR | O_NONBLOCK);
  Base(Base && other) noexcept;
  Base & operator=(Base && other) noexcept;
  ~Base();
  int query(const uint64_t & request, void *destination);

  std::string get_device_string();

protected:
  const std::string m_device_string;
  const char *m_device;
  int m_fd;

  std::string resolve_device_path(const std::string & device);

};

} // namespace device
} // namespace v4l

#endif // V4L__BASE_HPP_
