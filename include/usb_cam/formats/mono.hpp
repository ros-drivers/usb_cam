#ifndef USB_CAM__FORMATS__MONO_HPP_
#define USB_CAM__FORMATS__MONO_HPP_

#include "usb_cam/formats/pixel_format_base.hpp"
#include "usb_cam/formats/utils.hpp"


namespace usb_cam
{
namespace formats
{

class MONO8: public pixel_format_base
{
public:
    MONO8()
    : pixel_format_base(
        "mono8",
        V4L2_PIX_FMT_GREY,
        usb_cam::constants::MONO8,
        1,
        8,
        false)
    {}
};

class MONO16: public pixel_format_base
{
public:
    MONO16()
    : pixel_format_base(
        "mono16",
        V4L2_PIX_FMT_Y16,
        usb_cam::constants::MONO16,
        1,
        16,
        false)
    {}
};

}  // namespace formats
}  // namespace usb_cam

#endif  // USB_CAM__FORMATS__MONO_HPP_