#ifndef USB_CAM__DEVICE_HPP_
#define USB_CAM__DEVICE_HPP_
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

#include "usb_cam/constants.hpp"
#include "usb_cam/device/capability.hpp"
#include "usb_cam/device/input.hpp"


namespace usb_cam
{

    using ::usb_cam::constants::DIVIDER;

    /// @brief A RAII class for interacting with a V4L2 device
    class V4L2Device
    {

    public:
        V4L2Device(const std::string &device, int flags = O_RDWR | O_NONBLOCK);
        V4L2Device(V4L2Device &&other) noexcept;
        V4L2Device &operator=(V4L2Device &&other) noexcept;

        ~V4L2Device();

        int query(const uint64_t &request, void *destination);

    private:
        const char *m_device;
        int m_fd;

        /// @brief overview of the capabilities of this device
        device::capability m_capability;

        /// @brief the current input of this device
        device::input m_input;

        /// @brief the current output of this device
        struct v4l2_output m_video_output;

        /// @brief the current audio input
        struct v4l2_audio m_audio_input;

        std::string resolve_device_path(const std::string &device);

        /// @brief get the devices capabilities and store them in m_capability
        void get_capabilities();

        /// @brief get the devices current video input and store it in m_video_input
        void get_current_video_input();
    };

} // namespace usb_cam

#endif // USB_CAM__DEVICE_HPP_
