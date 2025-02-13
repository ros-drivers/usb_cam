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

#include "usb_cam/constants.hpp"
#include "usb_cam/device.hpp"

namespace usb_cam
{

V4L2Device::V4L2Device(const std::string &device, int flags)
: m_device(resolve_device_path(device).c_str()),
  m_fd(::open(m_device, flags))
{
    if (m_fd == -1)
    {
        auto err_str = "Failed to open device: " + device;
        err_str += " [resolved to `" + std::string(m_device) + "`]\n";
        err_str += std::string(strerror(errno));
        // TOOD: list possible devices;
        throw std::runtime_error(err_str.c_str());
    }

    std::cout << DIVIDER << std::endl;
    std::cout << "Opened device: " << device << std::endl;

    get_capabilities();
    m_capability.print();

    // Get this devices inputs and outputs if available
    get_current_video_input();
    m_input.print();

    // int output_index;
    // if (query(VIDIOC_G_OUTPUT, &output_index) == -1)
    // {
    //     auto err_str = "Could not retrieve device output index: " + std::string(m_device);
    //     throw std::runtime_error(err_str);
    // }
    // memset(&m_output, 0, sizeof(m_output));

    // m_output.index = output_index;
    // if (query(VIDIOC_ENUMOUTPUT, &m_output) == -1)
    // {
    //     auto err_str = "Could not retrieve device output: " + std::string(m_device);
    //     throw std::runtime_error(err_str);
    // }

}

V4L2Device::V4L2Device(V4L2Device &&other) noexcept : m_fd(other.m_fd)
{
    other.m_fd = -1; // Prevent double close
}

V4L2Device & V4L2Device::operator=(V4L2Device &&other) noexcept
{
    if (this != &other)
    {
        ::close(m_fd);
        m_fd = other.m_fd;
        other.m_fd = -1;
    }
    return *this;
}

V4L2Device::~V4L2Device()
{
    if (m_fd != -1)
    {
        ::close(m_fd);
    }
}

int V4L2Device::query(const uint64_t &request, void *destination)
{
    int result = 0;
    do
    {
        result = ::ioctl(m_fd, request, destination);
        continue;
    } while (-1 == result && EINTR == errno);
    return result;
}

std::string V4L2Device::resolve_device_path(const std::string &device)
{
    if (std::filesystem::is_symlink(device))
    {
        std::filesystem::path target_path = std::filesystem::read_symlink(device);
        // if the target path is relative, resolve it
        if (target_path.is_relative())
        {
            target_path = std::filesystem::absolute(device).parent_path() / target_path;
            target_path = std::filesystem::canonical(target_path);
        }
        return target_path.string();
    }
    return device;
}

void V4L2Device::get_capabilities()
{
    if (query(VIDIOC_QUERYCAP, &m_capability) == -1)
    {
        auto err_str = "Could not retrieve device capabilities: `" + std::string(m_device);
        throw std::runtime_error(err_str);
    }
}

void V4L2Device::get_current_video_input()
{
    int input_index;
    if (query(VIDIOC_G_INPUT, &input_index) == -1)
    {
        std::cout << "No video input for device: " + std::string(m_device) << std::endl;
        return;
    }
    memset(&m_input, 0, sizeof(m_input));
    m_input.index = input_index;
    if (query(VIDIOC_ENUMINPUT, &m_input) == -1)
    {
        auto err_str = "Could not retrieve device input: " + input_index;
        throw std::runtime_error(err_str);
    }
}

} // namespace usb_cam
