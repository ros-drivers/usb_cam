#include <iostream>
#include "linux/videodev2.h"

#include "usb_cam/device/input.hpp"


namespace usb_cam
{
namespace device
{


void input::print()
{
    std::cout << "Current video input:" << std::endl;
    std::cout << "  Name: " << name << std::endl;
    std::cout << "  Index: " << index << std::endl;
    std::cout << "  Type:  ";
    if (is_tuner()) { std::cout << "tuner" << std::endl; }
    if (is_camera()) { std::cout << "camera" << std::endl; }
    if (is_touch()) { std::cout << "touch" << std::endl; }
    std::cout << "  Status: " << std::endl;
    std::cout << "    - power: ";
    if (!has_power()) {
        // Don't worry about the other status flags if the power is off
        std::cout << "off" << std::endl;
     } else {
        std::cout << "on" << std::endl;
        std::cout << "    - signal: ";
        if (has_signal()) { std::cout << "on" << std::endl; }
        else { std::cout << "off" << std::endl; }
        std::cout << "    - color: ";
        if (has_color()) { std::cout << "on" << std::endl; }
        else { std::cout << "off" << std::endl; }
        std::cout << "    - flipped horizontally: ";
        if (is_flipped_horizontally()) { std::cout << "yes" << std::endl; }
        else { std::cout << "no" << std::endl; }
        std::cout << "    - flipped vertically: ";
        if (is_flipped_vertically()) { std::cout << "yes" << std::endl; }
        else { std::cout << "no" << std::endl; }
    }
    std::cout << "  Audioset: " << audioset << std::endl;
    std::cout << "  Tuner: " << tuner << std::endl;
    std::cout << "  Standard ID: " << std << std::endl;
}

bool input::is_tuner()
{
    return type == V4L2_INPUT_TYPE_TUNER;

}
bool input::is_camera()
{
    return type == V4L2_INPUT_TYPE_CAMERA;
}
bool input::is_touch()
{
    return type == V4L2_INPUT_TYPE_TOUCH;
}

bool input::has_power()
{
    return !(status & V4L2_IN_ST_NO_POWER);
}
bool input::has_signal()
{
    return !(status & V4L2_IN_ST_NO_SIGNAL);
}
bool input::has_color()
{
    return !(status & V4L2_IN_ST_NO_COLOR);
}

bool input::is_flipped_horizontally()
{
    return status & V4L2_IN_ST_HFLIP;
}

bool input::is_flipped_vertically()
{
    return status & V4L2_IN_ST_VFLIP;
}

bool input::has_horizontal_sync_lock()
{
    return false;
}
bool input::has_vertical_sync_lock()
{
    return false;
}

bool input::is_color_killer_active()
{
    return false;
}
bool input::has_standard_format_lock()
{
    return false;
}

bool input::has_sync_lock()
{
    return false;
}
bool input::has_equalizer_lock()
{
    return false;
}
bool input::has_carrier()
{
    return false;
}

bool input::has_macrovision()
{
    return false;
}
bool input::has_access()
{
    return false;
}
bool input::has_vtr_time_constant()
{
    return false;
}

bool input::supports_dv_timings()
{
    return false;
}
bool input::supports_std()
{
    return false;
}
bool input::supports_native_size()
{
    return false;
}

}  // namespace device
}  // namespace usb_cam
