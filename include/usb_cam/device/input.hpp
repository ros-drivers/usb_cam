#ifndef USB_CAM__DEVICE__VIDEO_IO_HPP
#define USB_CAM__DEVICE__VIDEO_IO_HPP

#include "linux/videodev2.h"


namespace usb_cam
{
namespace device
{


/// @brief Extend the v4l2_input struct with helper methods
struct input : v4l2_input
{
    void print();

    bool is_tuner();
    bool is_camera();
    bool is_touch();

    bool has_power();
    bool has_signal();
    bool has_color();

    bool is_flipped_horizontally();
    bool is_flipped_vertically();

    bool has_horizontal_sync_lock();
    bool has_vertical_sync_lock();

    bool is_color_killer_active();
    bool has_standard_format_lock();

    bool has_sync_lock();
    bool has_equalizer_lock();
    bool has_carrier();

    bool has_macrovision();
    bool has_access();
    bool has_vtr_time_constant();

    bool supports_dv_timings();
    bool supports_std();
    bool supports_native_size();
};

}  // namespace device
}  // namespace usb_cam

#endif  // USB_CAM__DEVICE__VIDEO_IO_HPP
