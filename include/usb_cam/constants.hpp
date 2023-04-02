// Copyright 2023 Evan Flynn
// Copyright 2014 Robert Bosch, LLC
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


#ifndef USB_CAM__CONSTANTS_HPP_
#define USB_CAM__CONSTANTS_HPP_

#include <string>
#include <vector>


namespace usb_cam
{
namespace constants
{


/// @brief image encodings duplicated from
/// https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/include/sensor_msgs/image_encodings.hpp
const char RGB8[] = "rgb8";
const char RGBA8[] = "rgba8";
const char RGB16[] = "rgb16";
const char RGBA16[] = "rgba16";
const char BGR8[] = "bgr8";
const char BGRA8[] = "bgra8";
const char BGR16[] = "bgr16";
const char BGRA16[] = "bgra16";
const char MONO8[] = "mono8";
const char MONO16[] = "mono16";

// OpenCV CvMat types
const char TYPE_8UC1[] = "8UC1";
const char TYPE_8UC2[] = "8UC2";
const char TYPE_8UC3[] = "8UC3";
const char TYPE_8UC4[] = "8UC4";
const char TYPE_8SC1[] = "8SC1";
const char TYPE_8SC2[] = "8SC2";
const char TYPE_8SC3[] = "8SC3";
const char TYPE_8SC4[] = "8SC4";
const char TYPE_16UC1[] = "16UC1";
const char TYPE_16UC2[] = "16UC2";
const char TYPE_16UC3[] = "16UC3";
const char TYPE_16UC4[] = "16UC4";
const char TYPE_16SC1[] = "16SC1";
const char TYPE_16SC2[] = "16SC2";
const char TYPE_16SC3[] = "16SC3";
const char TYPE_16SC4[] = "16SC4";
const char TYPE_32SC1[] = "32SC1";
const char TYPE_32SC2[] = "32SC2";
const char TYPE_32SC3[] = "32SC3";
const char TYPE_32SC4[] = "32SC4";
const char TYPE_32FC1[] = "32FC1";
const char TYPE_32FC2[] = "32FC2";
const char TYPE_32FC3[] = "32FC3";
const char TYPE_32FC4[] = "32FC4";
const char TYPE_64FC1[] = "64FC1";
const char TYPE_64FC2[] = "64FC2";
const char TYPE_64FC3[] = "64FC3";
const char TYPE_64FC4[] = "64FC4";

// Bayer encodings
const char BAYER_RGGB8[] = "bayer_rggb8";
const char BAYER_BGGR8[] = "bayer_bggr8";
const char BAYER_GBRG8[] = "bayer_gbrg8";
const char BAYER_GRBG8[] = "bayer_grbg8";
const char BAYER_RGGB16[] = "bayer_rggb16";
const char BAYER_BGGR16[] = "bayer_bggr16";
const char BAYER_GBRG16[] = "bayer_gbrg16";
const char BAYER_GRBG16[] = "bayer_grbg16";

// Miscellaneous
// YUV 4:2:2 encodings with an 8-bit depth
// UYUV version: http://www.fourcc.org/pixel-format/yuv-uyvy
const char YUV422[] = "yuv422";
// YUYV version: http://www.fourcc.org/pixel-format/yuv-yuy2/
const char YUV422_YUY2[] = "yuv422_yuy2";
// YUV 4:2:0 encodings with an 8-bit depth
// NV21: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/pixfmt-yuv-planar.html
const char NV21[] = "nv21";
// YUV 4:4:4 encodings with 8-bit depth
// NV24: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/pixfmt-yuv-planar.html
const char NV24[] = "nv24";

const char UNKNOWN[] = "unknown";

const std::vector<unsigned char> uchar_clipping_table = {
  0, 0, 0, 0, 0, 0, 0, 0,        // -128 - -121
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // -120 - -101
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // -100 - -81
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // -80  - -61
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // -60  - -41
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // -40  - -21
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // -20  - -1
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
  21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
  41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
  61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80,
  81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100,
  101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115,
  116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130,
  131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145,
  146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160,
  161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175,
  176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189, 190,
  191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205,
  206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220,
  221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235,
  236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250,
  251, 252, 253, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255,  // 256-263
  255, 255, 255, 255, 255, 255, 255, 255,  // 264-271
  255, 255, 255, 255, 255, 255, 255, 255,  // 272-279
  255, 255, 255, 255, 255, 255, 255, 255,  // 280-287
  255, 255, 255, 255, 255, 255, 255, 255,  // 288-295
  255, 255, 255, 255, 255, 255, 255, 255,  // 296-303
  255, 255, 255, 255, 255, 255, 255, 255,  // 304-311
  255, 255, 255, 255, 255, 255, 255, 255,  // 312-319
  255, 255, 255, 255, 255, 255, 255, 255,  // 320-327
  255, 255, 255, 255, 255, 255, 255, 255,  // 328-335
  255, 255, 255, 255, 255, 255, 255, 255,  // 336-343
  255, 255, 255, 255, 255, 255, 255, 255,  // 344-351
  255, 255, 255, 255, 255, 255, 255, 255,  // 352-359
  255, 255, 255, 255, 255, 255, 255, 255,  // 360-367
  255, 255, 255, 255, 255, 255, 255, 255,  // 368-375
  255, 255, 255, 255, 255, 255, 255, 255,  // 376-383
};
const int clipping_table_offset = 128;

}  // namespace constants
}  // namespace usb_cam

#endif  // USB_CAM__CONSTANTS_HPP_
