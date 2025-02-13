/// @brief Structures related to options for the UsbCam library
#ifndef USB_CAM__OPTIONS__STRUCTS_HPP_
#define USB_CAM__OPTIONS__STRUCTS_HPP_

#include <string>
#include <vector>
#include <functional>
#include <iostream>


namespace usb_cam
{

typedef std::string option_flag_t;
typedef std::vector<option_flag_t> options_t;
typedef std::vector<std::string> input_args_t;
typedef std::map<option_flag_t, std::string> options_with_values_t;

const options_t UsbCamOptions = {
    "--device"
};

}  // namespace usb_cam

#endif  // USB_CAM__OPTIONS__STRUCTS_HPP_
