#ifndef USB_CAM__FORMATS__YUYV_HPP_
#define USB_CAM__FORMATS__YUYV_HPP_

#include "usb_cam/formats/pixel_format_base.hpp"
#include "usb_cam/formats/utils.hpp"


namespace usb_cam
{
namespace formats
{

class YUYV: public pixel_format_base
{
public:
    YUYV()
    : pixel_format_base(
        "yuyv",
        V4L2_PIX_FMT_YUYV,
        usb_cam::constants::YUV422_YUY2,
        2,
        8,
        false)
    {}
};


class YUYV2RGB: public pixel_format_base
{
public:
    YUYV2RGB(const int & number_of_pixels)
    : pixel_format_base(
        "yuyv2rgb",
        V4L2_PIX_FMT_YUYV,
        usb_cam::constants::RGB8,
        3,
        8,
        true),
     m_number_of_pixels(number_of_pixels)
    {}

    /// @brief In this format each four bytes is two pixels. Each four bytes is two Y's, a Cb and a Cr.
    /// Each Y goes to one of the pixels, and the Cb and Cr belong to both pixels. As you can see, the
    /// Cr and Cb components have half the horizontal resolution of the Y component. V4L2_PIX_FMT_YUYV
    /// is known in the Windows environment as YUY2.
    ///
    /// Source: https://www.linuxtv.org/downloads/v4l-dvb-apis-old/V4L2-PIX-FMT-YUYV.html
    ///
    void convert(const char * & src, char * & dest, const int & bytes_used) override
    {
      (void)bytes_used;  // not used by this conversion method
      int i, j;
      unsigned char y0, y1, u, v;
      unsigned char r, g, b;

      /// Total number of bytes should be 2 * number of pixels. Achieve this by bit-shifting
      /// (NumPixels << 1). See format description above.
      for (i = 0, j = 0; i < (m_number_of_pixels << 1); i += 4, j += 6) {
        y0 = (unsigned char)src[i + 0];
        u = (unsigned char)src[i + 1];
        y1 = (unsigned char)src[i + 2];
        v = (unsigned char)src[i + 3];
        YUV2RGB(y0, u, v, &r, &g, &b);
        dest[j + 0] = r;
        dest[j + 1] = g;
        dest[j + 2] = b;
        YUV2RGB(y1, u, v, &r, &g, &b);
        dest[j + 3] = r;
        dest[j + 4] = g;
        dest[j + 5] = b;
      }
      return;
    }

private:
    int m_number_of_pixels;
};

}  // namespace formats
}  // namespace usb_cam

#endif  // USB_CAM__FORMATS__YUYV_HPP_
