#ifndef USB_CAM__FORMATS__RGB_HPP_
#define USB_CAM__FORMATS__RGB_HPP_

#include "linux/videodev2.h"

#include "usb_cam/formats/pixel_format_base.hpp"
#include "usb_cam/formats/utils.hpp"


namespace usb_cam
{
namespace formats
{

class RGB8: public pixel_format_base
{
public:
    RGB8()
    : pixel_format_base(
        "rgb8",
        V4L2_PIX_FMT_RGB332,
        usb_cam::constants::RGB8,
        3,
        8,
        false)
    {}
};

}  // namespace formats
}  // namespace usb_cam

#endif  // USB_CAM__FORMATS__RGB_HPP_
