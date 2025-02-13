/// @brief A pure C++ v4l2 CLI using the UsbCam library
#ifndef USB_CAM__CLI_HPP_
#define USB_CAM__CLI_HPP_

#include "usb_cam/options/errors.hpp"
#include "usb_cam/options/structs.hpp"
#include "usb_cam/device.hpp"

namespace usb_cam
{

class Cli {
public:
    Cli(input_args_t & args);
    ~Cli();

private:
    options_with_values_t parse_args(input_args_t & args);

    options_with_values_t m_settings;
    V4L2Device m_device;
};

} // namespace usb_cam


#endif  // USB_CAM__CLI_HPP_
