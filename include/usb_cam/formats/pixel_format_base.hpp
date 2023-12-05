// Copyright 2023 Evan Flynn
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Evan Flynn nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef USB_CAM__FORMATS__PIXEL_FORMAT_BASE_HPP_
#define USB_CAM__FORMATS__PIXEL_FORMAT_BASE_HPP_

#include <string>

#include "linux/videodev2.h"

#include "usb_cam/constants.hpp"
#include "usb_cam/conversions.hpp"

namespace usb_cam
{
namespace formats
{


/// @brief Helper structure to standardize all pixel_format_base
/// constructors, so that they all have the same argument type.
///
/// This should also be easily extendable in the future if we need to add additional
/// arguments for future pixel format(s) that are added.
typedef struct
{
  std::string name = "";
  int width = 640;
  int height = 480;
  size_t pixels = 640 * 480;
  std::string av_device_format_str = "AV_PIX_FMT_YUV422P";
} format_arguments_t;


/// @brief Base pixel format class. Provide all necessary information for converting between
/// V4L2 and ROS formats. Meant to be overridden if conversion function is required.
class pixel_format_base
{
public:
  pixel_format_base(
    std::string name, uint32_t v4l2, std::string ros,
    uint8_t channels, uint8_t bit_depth, bool requires_conversion)
  : m_name(name),
    m_v4l2(v4l2),
    m_ros(ros),
    m_channels(channels),
    m_bit_depth(bit_depth),
    m_requires_conversion(requires_conversion)
  {}

  /// @brief Name of pixel format. Used in the parameters file to select this format
  /// @return
  inline std::string name() {return m_name;}

  /// @brief Integer value of V4L2 capture pixel format
  /// @return uint32_t V4L2 capture pixel format
  inline uint32_t v4l2() {return m_v4l2;}

  /// @brief String value of V4L2 capture pixel format
  /// @return std::string V4L2 capture pixel format
  inline std::string v4l2_str() {return usb_cam::conversions::FCC2S(m_v4l2);}

  /// @brief Name of output pixel (encoding) format to ROS
  /// @return
  inline std::string ros() {return m_ros;}

  /// @brief Number of channels (e.g. bytes) per pixel
  /// @return
  inline uint8_t channels() {return m_channels;}

  /// @brief Number for bit depth of image
  /// @return
  inline uint8_t bit_depth() {return m_bit_depth;}

  /// @brief Number of bytes per channel
  inline uint8_t byte_depth() {return m_bit_depth / 8;}

  /// @brief True if the current pixel format requires a call to the `convert` method
  /// Used in the usb_cam library logic to determine if a plain `memcopy` call can be
  /// used instead of a call to the `convert` method of this class.
  /// @return
  inline bool requires_conversion() {return m_requires_conversion;}

  /// @brief Conversion method. Meant to be overridden if pixel format requires it.
  virtual void convert(const char * & src, char * & dest, const int & bytes_used)
  {
    // provide default implementation so derived classes do not have to implement
    // this method if not required
    (void)src;
    (void)dest;
    (void)bytes_used;
  }

  /// @brief Returns if the final output format is color
  /// Copied from:
  ///     https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/include/sensor_msgs/image_encodings.hpp
  /// @return
  inline bool is_color()
  {
    return
      m_ros == usb_cam::constants::RGB8 ||
      m_ros == usb_cam::constants::BGR8 ||
      m_ros == usb_cam::constants::RGBA8 ||
      m_ros == usb_cam::constants::BGRA8 ||
      m_ros == usb_cam::constants::RGB16 ||
      m_ros == usb_cam::constants::BGR16 ||
      m_ros == usb_cam::constants::RGBA16 ||
      m_ros == usb_cam::constants::BGRA16 ||
      m_ros == usb_cam::constants::NV21 ||
      m_ros == usb_cam::constants::NV24;
  }

  /// @brief Returns if the final output format is monocolor (gray)
  /// Copied from:
  ///     https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/include/sensor_msgs/image_encodings.hpp
  /// @return
  inline bool is_mono()
  {
    return
      m_ros == usb_cam::constants::MONO8 ||
      m_ros == usb_cam::constants::MONO16;
  }

  /// @brief Returns if the final output format is bayer
  /// Copied from:
  ///     https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/include/sensor_msgs/image_encodings.hpp
  /// @return
  inline bool is_bayer()
  {
    return
      m_ros == usb_cam::constants::BAYER_RGGB8 ||
      m_ros == usb_cam::constants::BAYER_BGGR8 ||
      m_ros == usb_cam::constants::BAYER_GBRG8 ||
      m_ros == usb_cam::constants::BAYER_GRBG8 ||
      m_ros == usb_cam::constants::BAYER_RGGB16 ||
      m_ros == usb_cam::constants::BAYER_BGGR16 ||
      m_ros == usb_cam::constants::BAYER_GBRG16 ||
      m_ros == usb_cam::constants::BAYER_GRBG16;
  }

  /// @brief Returns if the final output format has an alpha value
  /// Copied from:
  ///     https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/include/sensor_msgs/image_encodings.hpp
  /// @return
  inline bool has_alpha()
  {
    return
      m_ros == usb_cam::constants::RGBA8 ||
      m_ros == usb_cam::constants::BGRA8 ||
      m_ros == usb_cam::constants::RGBA16 ||
      m_ros == usb_cam::constants::BGRA16;
  }

protected:
  /// @brief Unique name for this pixel format
  std::string m_name;
  /// @brief Integer correspoding to a specific V4L2_PIX_FMT_* constant
  /// See `linux/videodev2.h` for a list of all possible values for here
  uint32_t m_v4l2;
  // TODO(flynneva): make this a vector of supported conversions for the specified V4L2 format
  /// @brief This should match ROS encoding string
  /// See `sensor_msgs/image_encodings.hpp` for corresponding possible values. Copy of
  /// those values are stored in `usb_cam/constants.hpp`
  std::string m_ros;
  /// @brief Number of channels (aka bytes per pixel) of output (ROS format above)
  uint8_t m_channels;
  /// @brief Bitdepth of output (ROS format above)
  uint8_t m_bit_depth;
  /// @brief boolean whether or not the current format requires a call to `convert`.
  /// Setting this to true requires that the virtual `convert` method is implemented.
  bool m_requires_conversion;
};


class default_pixel_format : public pixel_format_base
{
public:
  default_pixel_format()
  : pixel_format_base(
      "yuyv",
      V4L2_PIX_FMT_YUYV,
      usb_cam::constants::YUV422_YUY2,
      2,
      8,
      false)
  {}
};

}  // namespace formats
}  // namespace usb_cam

#endif  // USB_CAM__FORMATS__PIXEL_FORMAT_BASE_HPP_
