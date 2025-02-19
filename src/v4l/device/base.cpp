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
#include "v4l/device/base.hpp"
#include "v4l/errors.hpp"


namespace v4l
{
namespace device
{

Base::Base()
: m_device_string("/dev/video0"),
  m_device(resolve_device_path(m_device_string).c_str()),
  m_fd(::open(m_device, 0))
{}

Base::Base(const std::string & device, int flags)
: m_device_string(device),
  m_device(resolve_device_path(m_device_string).c_str()),
  m_fd(::open(m_device, flags))
{
  if (m_fd == -1) {
    auto err_str = device + " [resolved to `" + std::string(m_device) + "`]\n";
        // TOOD: list possible devices;
    throw FailedToOpen(err_str.c_str());
  }
    // device is open
}

Base::Base(Base && other) noexcept
: m_fd(other.m_fd)
{
  other.m_fd = -1;   // Prevent double close
}

Base & Base::operator=(Base && other) noexcept
{
  if (this != &other) {
    ::close(m_fd);
    m_fd = other.m_fd;
    other.m_fd = -1;
  }
  return *this;
}

Base::~Base()
{
  if (m_fd != -1) {
    ::close(m_fd);
  }
}

std::string Base::get_device_string()
{
  return m_device_string;
}

int Base::query(const uint64_t & request, void *destination)
{
  int result = 0;
  do{
    try {
      result = ::ioctl(m_fd, request, destination);
    } catch (const std::exception & err) {
      throw QueryError();
    }
    continue;
  } while (-1 == result && EINTR == errno);
  return result;
}

std::string Base::resolve_device_path(const std::string & device)
{
  if (std::filesystem::is_symlink(device)) {
    std::filesystem::path target_path = std::filesystem::read_symlink(device);
        // if the target path is relative, resolve it
    if (target_path.is_relative()) {
      target_path = std::filesystem::absolute(device).parent_path() / target_path;
      target_path = std::filesystem::canonical(target_path);
    }
    return target_path.string();
  }
  return device;
}

}  // namespace device
}  // namespace v4l
