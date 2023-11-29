// Copyright 2023 Boitumelo Ruf
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

#ifndef USB_CAM__FORMATS__AV_PIXEL_FORMAT_HELPER_HPP_
#define USB_CAM__FORMATS__AV_PIXEL_FORMAT_HELPER_HPP_

#include <unordered_map>
#include <algorithm>
#include <string>

extern "C" {
#define __STDC_CONSTANT_MACROS  // Required for libavutil
#include "libavutil/pixfmt.h"
}

#include "usb_cam/constants.hpp"

#define stringify(name) #name


namespace usb_cam
{
namespace formats
{

/// Map to associate string of pixel format name to actual pixel format enum
const std::unordered_map<std::string, AVPixelFormat> STR_2_AVPIXFMT = {
  {stringify(AV_PIX_FMT_NONE), AV_PIX_FMT_NONE},

  {stringify(AV_PIX_FMT_YUV420P), AV_PIX_FMT_YUV420P},        ///< planar YUV 4:2:0, 12bpp,
                                                              ///< (1 Cr & Cb sample per 2x2 Y
                                                              ///< samples)

  {stringify(AV_PIX_FMT_YUYV422), AV_PIX_FMT_YUYV422},        ///< packed YUV 4:2:2, 16bpp,
                                                              ///< Y0 Cb Y1 Cr

  {stringify(AV_PIX_FMT_RGB24), AV_PIX_FMT_RGB24},            ///< packed RGB 8:8:8, 24bpp,
                                                              ///< RGBRGB...

  {stringify(AV_PIX_FMT_BGR24), AV_PIX_FMT_BGR24},            ///< packed RGB 8:8:8, 24bpp,
                                                              ///< BGRBGR...

  {stringify(AV_PIX_FMT_YUV422P), AV_PIX_FMT_YUV422P},        ///< planar YUV 4:2:2, 16bpp,
                                                              ///< (1 Cr & Cb sample per 2x1 Y
                                                              ///< samples)

  {stringify(AV_PIX_FMT_YUV444P), AV_PIX_FMT_YUV444P},        ///< planar YUV 4:4:4, 24bpp, (1 Cr
                                                              ///< & Cb sample per 1x1 Y samples)

  {stringify(AV_PIX_FMT_YUV410P), AV_PIX_FMT_YUV410P},        ///< planar YUV 4:1:0,  9bpp, (1 Cr
                                                              ///< & Cb sample per 4x4 Y samples)

  {stringify(AV_PIX_FMT_YUV411P), AV_PIX_FMT_YUV411P},        ///< planar YUV 4:1:1, 12bpp, (1 Cr
                                                              ///< & Cb sample per 4x1 Y samples)

  {stringify(AV_PIX_FMT_GRAY8), AV_PIX_FMT_GRAY8},            ///<        Y        ,  8bpp

  {stringify(AV_PIX_FMT_MONOWHITE), AV_PIX_FMT_MONOWHITE},    ///<        Y        ,  1bpp, 0 is
                                                              ///< white, 1 is black, in each byte
                                                              ///< pixels are ordered from the msb
                                                              ///< to the lsb

  {stringify(AV_PIX_FMT_MONOBLACK), AV_PIX_FMT_MONOBLACK},    ///<        Y        ,  1bpp, 0 is
                                                              ///< black, 1 is white, in each byte
                                                              ///< pixels are ordered from the msb
                                                              ///< to the lsb

  {stringify(AV_PIX_FMT_PAL8), AV_PIX_FMT_PAL8},              ///< 8 bits with AV_PIX_FMT_RGB32
                                                              ///< palette

  {stringify(AV_PIX_FMT_YUVJ420P), AV_PIX_FMT_YUVJ420P},      ///< planar YUV 4:2:0, 12bpp, full
                                                              ///< scale (JPEG), deprecated in
                                                              ///< favor of AV_PIX_FMT_YUV420P and
                                                              ///< setting color_range

  {stringify(AV_PIX_FMT_YUVJ422P), AV_PIX_FMT_YUVJ422P},      ///< planar YUV 4:2:2, 16bpp, full
                                                              ///< scale (JPEG), deprecated in
                                                              ///< favor of AV_PIX_FMT_YUV422P and
                                                              ///< setting color_range

  {stringify(AV_PIX_FMT_YUVJ444P), AV_PIX_FMT_YUVJ444P},      ///< planar YUV 4:4:4, 24bpp,
                                                              ///< full scale (JPEG), deprecated
                                                              ///< in favor of AV_PIX_FMT_YUV444P
                                                              ///< and setting color_range

  {stringify(AV_PIX_FMT_UYVY422), AV_PIX_FMT_UYVY422},        ///< packed YUV 4:2:2, 16bpp, Cb Y0
                                                              ///< Cr Y1

  {stringify(AV_PIX_FMT_UYYVYY411), AV_PIX_FMT_UYYVYY411},    ///< packed YUV 4:1:1, 12bpp, Cb Y0
                                                              ///< Y1 Cr Y2 Y3

  {stringify(AV_PIX_FMT_BGR8), AV_PIX_FMT_BGR8},              ///< packed RGB 3:3:2,  8bpp,
                                                              ///< (msb)2B 3G 3R(lsb)

  {stringify(AV_PIX_FMT_BGR4), AV_PIX_FMT_BGR4},              ///< packed RGB 1:2:1 bitstream,
                                                              ///< 4bpp, (msb)1B 2G 1R(lsb), a
                                                              ///< byte contains two pixels, the
                                                              ///< first pixel in the byte is the
                                                              ///< one composed by the 4 msb bits

  {stringify(AV_PIX_FMT_BGR4_BYTE), AV_PIX_FMT_BGR4_BYTE},    ///< packed RGB 1:2:1,  8bpp,
                                                              ///< (msb)1B 2G 1R(lsb)

  {stringify(AV_PIX_FMT_RGB8), AV_PIX_FMT_RGB8},              ///< packed RGB 3:3:2,  8bpp,
                                                              ///< (msb)2R 3G 3B(lsb)

  {stringify(AV_PIX_FMT_RGB4), AV_PIX_FMT_RGB4},              ///< packed RGB 1:2:1 bitstream,
                                                              ///< 4bpp, (msb)1R 2G 1B(lsb), a
                                                              ///< byte contains two pixels, the
                                                              ///< first pixel in the byte is the
                                                              ///< one composed by the 4 msb bits

  {stringify(AV_PIX_FMT_RGB4_BYTE), AV_PIX_FMT_RGB4_BYTE},    ///< packed RGB 1:2:1,  8bpp,
                                                              ///< (msb)1R 2G 1B(lsb)

  {stringify(AV_PIX_FMT_NV12), AV_PIX_FMT_NV12},              ///< planar YUV 4:2:0, 12bpp, 1
                                                              ///< plane for Y and 1 plane for the
                                                              ///< UV components, which are
                                                              ///< interleaved (first byte U and
                                                              ///< the following byte V)

  {stringify(AV_PIX_FMT_NV21), AV_PIX_FMT_NV21},              ///< as above, but U and V bytes are
                                                              ///< swapped


  {stringify(AV_PIX_FMT_ARGB), AV_PIX_FMT_ARGB},              ///< packed ARGB 8:8:8:8, 32bpp,
                                                              ///< ARGBARGB...

  {stringify(AV_PIX_FMT_RGBA), AV_PIX_FMT_RGBA},              ///< packed RGBA 8:8:8:8, 32bpp,
                                                              ///< RGBARGBA...

  {stringify(AV_PIX_FMT_ABGR), AV_PIX_FMT_ABGR},              ///< packed ABGR 8:8:8:8, 32bpp,
                                                              ///< ABGRABGR...

  {stringify(AV_PIX_FMT_BGRA), AV_PIX_FMT_BGRA},              ///< packed BGRA 8:8:8:8, 32bpp,
                                                              ///< BGRABGRA...


  {stringify(AV_PIX_FMT_GRAY16BE), AV_PIX_FMT_GRAY16BE},      ///<        Y        , 16bpp,
                                                              ///< big-endian

  {stringify(AV_PIX_FMT_GRAY16LE), AV_PIX_FMT_GRAY16LE},      ///<        Y        , 16bpp,
                                                              ///< little-endian

  {stringify(AV_PIX_FMT_YUV440P), AV_PIX_FMT_YUV440P},        ///< planar YUV 4:4:0 (1 Cr & Cb
                                                              ///< sample per 1x2 Y samples)

  {stringify(AV_PIX_FMT_YUVJ440P), AV_PIX_FMT_YUVJ440P},      ///< planar YUV 4:4:0 full scale
                                                              ///< (JPEG), deprecated in favor of
                                                              ///< AV_PIX_FMT_YUV440P and setting
                                                              ///< color_range

  {stringify(AV_PIX_FMT_YUVA420P), AV_PIX_FMT_YUVA420P},      ///< planar YUV 4:2:0, 20bpp, (1 Cr
                                                              ///< & Cb sample per 2x2 Y & A
                                                              ///< samples)

  {stringify(AV_PIX_FMT_RGB48BE), AV_PIX_FMT_RGB48BE},        ///< packed RGB 16:16:16, 48bpp,
                                                              ///< 16R, 16G, 16B, the 2-byte value
                                                              ///< for each R/G/B component is
                                                              ///< stored as big-endian

  {stringify(AV_PIX_FMT_RGB48LE), AV_PIX_FMT_RGB48LE},        ///< packed RGB 16:16:16, 48bpp,
                                                              ///< 16R, 16G, 16B, the 2-byte value
                                                              ///< for each R/G/B component is
                                                              ///< stored as little-endian


  {stringify(AV_PIX_FMT_RGB565BE), AV_PIX_FMT_RGB565BE},      ///< packed RGB 5:6:5, 16bpp, (msb)
                                                              ///< 5R 6G 5B(lsb), big-endian

  {stringify(AV_PIX_FMT_RGB565LE), AV_PIX_FMT_RGB565LE},      ///< packed RGB 5:6:5, 16bpp, (msb)
                                                              ///< 5R 6G 5B(lsb), little-endian

  {stringify(AV_PIX_FMT_RGB555BE), AV_PIX_FMT_RGB555BE},      ///< packed RGB 5:5:5, 16bpp,
                                                              ///< (msb)1X 5R 5G 5B(lsb),
                                                              ///< big-endian , X=unused/undefined

  {stringify(AV_PIX_FMT_RGB555LE), AV_PIX_FMT_RGB555LE},      ///< packed RGB 5:5:5, 16bpp,
                                                              ///< (msb)1X 5R 5G 5B(lsb),
                                                              ///< little-endian,
                                                              ///< X=unused/undefined


  {stringify(AV_PIX_FMT_BGR565BE), AV_PIX_FMT_RGB565BE},      ///< packed BGR 5:6:5, 16bpp, (msb)
                                                              ///< 5B 6G 5R(lsb), big-endian

  {stringify(AV_PIX_FMT_BGR565LE), AV_PIX_FMT_RGB565LE},      ///< packed BGR 5:6:5, 16bpp, (msb)
                                                              ///< 5B 6G 5R(lsb), little-endian

  {stringify(AV_PIX_FMT_BGR555BE), AV_PIX_FMT_RGB555BE},      ///< packed BGR 5:5:5, 16bpp,
                                                              ///< (msb)1X 5B 5G 5R(lsb),
                                                              ///< big-endian, X=unused/undefined

  {stringify(AV_PIX_FMT_BGR555LE), AV_PIX_FMT_RGB555LE},      ///< packed BGR 5:5:5, 16bpp,
                                                              ///< (msb)1X 5B 5G 5R(lsb),
                                                              ///< little-endian,
                                                              ///< X=unused/undefined

#if FF_API_VAAPI
  /** @name Deprecated pixel formats */
  /**@{*/
  {stringify(AV_PIX_FMT_VAAPI_MOCO), AV_PIX_FMT_VAAPI_MOCO},  ///< HW acceleration through VA API
                                                              ///< at motion compensation
                                                              ///< entry-point, Picture.data[3]
                                                              ///< contains a vaapi_render_state
                                                              ///< struct which contains
                                                              ///< macroblocks as well as various
                                                              ///< fields extracted from headers

  {stringify(AV_PIX_FMT_VAAPI_IDCT), AV_PIX_FMT_VAAPI_IDCT},  ///< HW acceleration through VA API
                                                              ///< at IDCT entry-point,
                                                              ///< Picture.data[3] contains a
                                                              ///< vaapi_render_state struct which
                                                              ///< contains fields extracted from
                                                              ///< headers

  {stringify(AV_PIX_FMT_VAAPI_VLD), AV_PIX_FMT_VAAPI_VLD},    ///< HW decoding through VA API,
                                                              ///< Picture.data[3] contains a
                                                              ///< VASurfaceID
  /**@}*/
  {stringify(AV_PIX_FMT_VAAPI), AV_PIX_FMT_VAAPI},
#else
  /**
   *  Hardware acceleration through VA-API, data[3] contains a
   *  VASurfaceID.
   */
  {stringify(AV_PIX_FMT_VAAPI), AV_PIX_FMT_VAAPI},
#endif

  {stringify(AV_PIX_FMT_YUV420P16LE), AV_PIX_FMT_YUV420P16LE},    ///< planar YUV 4:2:0, 24bpp,
                                                                  ///< (1 Cr & Cb sample per 2x2
                                                                  ///< Y samples), little-endian

  {stringify(AV_PIX_FMT_YUV420P16BE), AV_PIX_FMT_YUV420P16BE},    ///< planar YUV 4:2:0, 24bpp,
                                                                  ///< (1 Cr & Cb sample per 2x2
                                                                  ///< Y samples), big-endian

  {stringify(AV_PIX_FMT_YUV422P16LE), AV_PIX_FMT_YUV422P16LE},    ///< planar YUV 4:2:2, 32bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y samples), little-endian

  {stringify(AV_PIX_FMT_YUV422P16BE), AV_PIX_FMT_YUV422P16BE},    ///< planar YUV 4:2:2, 32bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y samples), big-endian

  {stringify(AV_PIX_FMT_YUV444P16LE), AV_PIX_FMT_YUV444P16LE},    ///< planar YUV 4:4:4, 48bpp,
                                                                  ///< (1 Cr & Cb sample per 1x1
                                                                  ///< Y samples), little-endian

  {stringify(AV_PIX_FMT_YUV444P16BE), AV_PIX_FMT_YUV444P16BE},    ///< planar YUV 4:4:4, 48bpp,
                                                                  ///< (1 Cr & Cb sample per 1x1
                                                                  ///< Y samples), big-endian

  {stringify(AV_PIX_FMT_DXVA2_VLD), AV_PIX_FMT_DXVA2_VLD},        ///< HW decoding through DXVA2,
                                                                  ///< Picture.data[3] contains a
                                                                  ///< LPDIRECT3DSURFACE9 pointer


  {stringify(AV_PIX_FMT_RGB444LE), AV_PIX_FMT_RGB444LE},          ///< packed RGB 4:4:4, 16bpp,
                                                                  ///< (msb)4X 4R 4G 4B(lsb),
                                                                  ///< little-endian,
                                                                  ///< X=unused/undefined

  {stringify(AV_PIX_FMT_RGB444BE), AV_PIX_FMT_RGB444BE},          ///< packed RGB 4:4:4, 16bpp,
                                                                  ///< (msb)4X 4R 4G 4B(lsb),
                                                                  ///< big-endian,
                                                                  ///< X=unused/undefined

  {stringify(AV_PIX_FMT_BGR444LE), AV_PIX_FMT_BGR444LE},          ///< packed BGR 4:4:4, 16bpp,
                                                                  ///< (msb)4X 4B 4G 4R(lsb),
                                                                  ///< little-endian,
                                                                  ///< X=unused/undefined

  {stringify(AV_PIX_FMT_BGR444BE), AV_PIX_FMT_BGR444BE},          ///< packed BGR 4:4:4, 16bpp,
                                                                  ///< (msb)4X 4B 4G 4R(lsb),
                                                                  ///< big-endian,
                                                                  ///< X=unused/undefined

  {stringify(AV_PIX_FMT_YA8), AV_PIX_FMT_YA8},                    ///< 8 bits gray, 8 bits alpha


  {stringify(AV_PIX_FMT_Y400A), AV_PIX_FMT_Y400A},                ///< alias for AV_PIX_FMT_YA8

  {stringify(AV_PIX_FMT_GRAY8A), AV_PIX_FMT_GRAY8A},              ///< alias for AV_PIX_FMT_YA8


  {stringify(AV_PIX_FMT_BGR48BE), AV_PIX_FMT_BGR48BE},            ///< packed RGB 16:16:16, 48bpp,
                                                                  ///< 16B, 16G, 16R, the 2-byte
                                                                  ///< value for each R/G/B
                                                                  ///< component is stored as
                                                                  ///< big-endian

  {stringify(AV_PIX_FMT_BGR48LE), AV_PIX_FMT_BGR48LE},            ///< packed RGB 16:16:16, 48bpp,
                                                                  ///< 16B, 16G, 16R, the 2-byte
                                                                  ///< value for each R/G/B
                                                                  ///< component is stored as
                                                                  ///< little-endian

  /**
   * The following 12 formats have the disadvantage of needing 1 format for each bit depth.
   * Notice that each 9/10 bits sample is stored in 16 bits with extra padding.
   * If you want to support multiple bit depths, then using AV_PIX_FMT_YUV420P16* with the bpp
   * stored separately is better.
   */
  {stringify(AV_PIX_FMT_YUV420P9BE), AV_PIX_FMT_YUV420P9BE},      ///< planar YUV 4:2:0, 13.5bpp,
                                                                  ///< (1 Cr & Cb sample per 2x2 Y
                                                                  ///< samples), big-endian

  {stringify(AV_PIX_FMT_YUV420P9LE), AV_PIX_FMT_YUV420P9LE},      ///< planar YUV 4:2:0, 13.5bpp,
                                                                  ///< (1 Cr & Cb sample per 2x2 Y
                                                                  ///< samples), little-endian

  {stringify(AV_PIX_FMT_YUV420P10BE), AV_PIX_FMT_YUV420P10BE},    ///< planar YUV 4:2:0, 15bpp,
                                                                  ///< (1 Cr & Cb sample per 2x2
                                                                  ///< Y samples), big-endian

  {stringify(AV_PIX_FMT_YUV420P10LE), AV_PIX_FMT_YUV420P10LE},    ///< planar YUV 4:2:0, 15bpp,
                                                                  ///< (1 Cr & Cb sample per 2x2
                                                                  ///< Y samples), little-endian

  {stringify(AV_PIX_FMT_YUV422P10BE), AV_PIX_FMT_YUV422P10BE},    ///< planar YUV 4:2:2, 20bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y samples), big-endian

  {stringify(AV_PIX_FMT_YUV422P10LE), AV_PIX_FMT_YUV422P10LE},    ///< planar YUV 4:2:2, 20bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y samples), little-endian

  {stringify(AV_PIX_FMT_YUV444P9BE), AV_PIX_FMT_YUV444P9BE},      ///< planar YUV 4:4:4, 27bpp,
                                                                  ///< (1 Cr & Cb sample per 1x1
                                                                  ///< Y samples), big-endian

  {stringify(AV_PIX_FMT_YUV444P9LE), AV_PIX_FMT_YUV444P9LE},      ///< planar YUV 4:4:4, 27bpp,
                                                                  ///< (1 Cr & Cb sample per 1x1
                                                                  ///< Y samples), little-endian

  {stringify(AV_PIX_FMT_YUV444P10BE), AV_PIX_FMT_YUV444P10BE},    ///< planar YUV 4:4:4, 30bpp,
                                                                  ///< (1 Cr & Cb sample per 1x1
                                                                  ///< Y samples), big-endian

  {stringify(AV_PIX_FMT_YUV444P10LE), AV_PIX_FMT_YUV444P10LE},    ///< planar YUV 4:4:4, 30bpp,
                                                                  ///< (1 Cr & Cb sample per 1x1
                                                                  ///< Y samples), little-endian

  {stringify(AV_PIX_FMT_YUV422P9BE), AV_PIX_FMT_YUV422P9BE},      ///< planar YUV 4:2:2, 18bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y samples), big-endian

  {stringify(AV_PIX_FMT_YUV422P9LE), AV_PIX_FMT_YUV422P9LE},      ///< planar YUV 4:2:2, 18bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y samples), little-endian

  {stringify(AV_PIX_FMT_GBRP), AV_PIX_FMT_GBRP},                  ///< planar GBR 4:4:4 24bpp

  {stringify(AV_PIX_FMT_GBR24P), AV_PIX_FMT_GBR24P},              // alias for #AV_PIX_FMT_GBRP

  {stringify(AV_PIX_FMT_GBRP9BE), AV_PIX_FMT_GBRP9BE},            ///< planar GBR 4:4:4 27bpp,
                                                                  ///< big-endian

  {stringify(AV_PIX_FMT_GBRP9LE), AV_PIX_FMT_GBRP9LE},            ///< planar GBR 4:4:4 27bpp,
                                                                  ///< little-endian

  {stringify(AV_PIX_FMT_GBRP10BE), AV_PIX_FMT_GBRP10BE},          ///< planar GBR 4:4:4 30bpp,
                                                                  ///< big-endian

  {stringify(AV_PIX_FMT_GBRP10LE), AV_PIX_FMT_GBRP10LE},          ///< planar GBR 4:4:4 30bpp,
                                                                  ///< little-endian

  {stringify(AV_PIX_FMT_GBRP16BE), AV_PIX_FMT_GBRP16BE},          ///< planar GBR 4:4:4 48bpp,
                                                                  ///< big-endian

  {stringify(AV_PIX_FMT_GBRP16LE), AV_PIX_FMT_GBRP16LE},          ///< planar GBR 4:4:4 48bpp,
                                                                  ///< little-endian

  {stringify(AV_PIX_FMT_YUVA422P), AV_PIX_FMT_YUVA422P},          ///< planar YUV 4:2:2 24bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y & A samples)

  {stringify(AV_PIX_FMT_YUVA444P), AV_PIX_FMT_YUVA444P},          ///< planar YUV 4:4:4 32bpp,
                                                                  ///< (1 Cr & Cb sample per 1x1
                                                                  ///< Y & A samples)

  {stringify(AV_PIX_FMT_YUVA420P9BE), AV_PIX_FMT_YUVA420P9BE},    ///< planar YUV 4:2:0 22.5bpp,
                                                                  ///< (1 Cr & Cb sample per 2x2
                                                                  ///< Y & A samples), big-endian

  {stringify(AV_PIX_FMT_YUVA420P9LE), AV_PIX_FMT_YUVA420P9LE},    ///< planar YUV 4:2:0 22.5bpp,
                                                                  ///< (1 Cr & Cb sample per 2x2
                                                                  ///< Y & A samples),
                                                                  ///< little-endian

  {stringify(AV_PIX_FMT_YUVA422P9BE), AV_PIX_FMT_YUVA422P9BE},    ///< planar YUV 4:2:2 27bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y & A samples), big-endian

  {stringify(AV_PIX_FMT_YUVA422P9LE), AV_PIX_FMT_YUVA422P9LE},    ///< planar YUV 4:2:2 27bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y & A samples),
                                                                  ///< little-endian

  {stringify(AV_PIX_FMT_YUVA444P9BE), AV_PIX_FMT_YUVA444P9BE},    ///< planar YUV 4:4:4 36bpp,
                                                                  ///< (1 Cr & Cb sample per 1x1
                                                                  ///< Y & A samples), big-endian

  {stringify(AV_PIX_FMT_YUVA444P9LE), AV_PIX_FMT_YUVA444P9LE},    ///< planar YUV 4:4:4 36bpp,
                                                                  ///< (1 Cr & Cb sample per 1x1
                                                                  ///< Y & A samples),
                                                                  ///< little-endian

  {stringify(AV_PIX_FMT_YUVA420P10BE), AV_PIX_FMT_YUVA420P10BE},  ///< planar YUV 4:2:0 25bpp,
                                                                  ///< (1 Cr & Cb sample per 2x2
                                                                  ///< Y & A samples, big-endian)

  {stringify(AV_PIX_FMT_YUVA420P10LE), AV_PIX_FMT_YUVA420P10LE},  ///< planar YUV 4:2:0 25bpp,
                                                                  ///< (1 Cr & Cb sample per 2x2
                                                                  ///< Y & A samples,
                                                                  ///< little-endian)

  {stringify(AV_PIX_FMT_YUVA422P10BE), AV_PIX_FMT_YUVA422P10BE},  ///< planar YUV 4:2:2 30bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y & A samples, big-endian)

  {stringify(AV_PIX_FMT_YUVA422P10LE), AV_PIX_FMT_YUVA422P10LE},  ///< planar YUV 4:2:2 30bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y & A samples,
                                                                  ///< little-endian)

  {stringify(AV_PIX_FMT_YUVA444P10BE), AV_PIX_FMT_YUVA444P10BE},  ///< planar YUV 4:4:4 40bpp,
                                                                  ///< (1 Cr & Cb sample per 1x1
                                                                  ///< Y & A samples, big-endian)

  {stringify(AV_PIX_FMT_YUVA444P10LE), AV_PIX_FMT_YUVA444P10LE},  ///< planar YUV 4:4:4 40bpp,
                                                                  ///< (1 Cr & Cb sample per 1x1
                                                                  ///< Y & A samples,
                                                                  ///< little-endian)

  {stringify(AV_PIX_FMT_YUVA420P16BE), AV_PIX_FMT_YUVA420P16BE},  ///< planar YUV 4:2:0 40bpp,
                                                                  ///< (1 Cr & Cb sample per 2x2
                                                                  ///< Y & A samples, big-endian)

  {stringify(AV_PIX_FMT_YUVA420P16LE), AV_PIX_FMT_YUVA420P16LE},  ///< planar YUV 4:2:0 40bpp,
                                                                  ///< (1 Cr & Cb sample per 2x2
                                                                  ///< Y & A samples,
                                                                  ///< little-endian)

  {stringify(AV_PIX_FMT_YUVA422P16BE), AV_PIX_FMT_YUVA422P16BE},  ///< planar YUV 4:2:2 48bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y & A samples, big-endian)

  {stringify(AV_PIX_FMT_YUVA422P16LE), AV_PIX_FMT_YUVA422P16LE},  ///< planar YUV 4:2:2 48bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y & A samples,
                                                                  ///< little-endian)

  {stringify(AV_PIX_FMT_YUVA444P16BE), AV_PIX_FMT_YUVA444P16BE},  ///< planar YUV 4:4:4 64bpp,
                                                                  ///< (1 Cr & Cb sample per 1x1
                                                                  ///< Y & A samples, big-endian)

  {stringify(AV_PIX_FMT_YUVA444P16LE), AV_PIX_FMT_YUVA444P16LE},  ///< planar YUV 4:4:4 64bpp,
                                                                  ///< (1 Cr & Cb sample per 1x1
                                                                  ///< Y & A samples,
                                                                  ///< little-endian)


  {stringify(AV_PIX_FMT_VDPAU), AV_PIX_FMT_VDPAU},                ///< HW acceleration through
                                                                  ///< VDPAU, Picture.data[3]
                                                                  ///< contains a VdpVideoSurface


  {stringify(AV_PIX_FMT_XYZ12LE), AV_PIX_FMT_XYZ12LE},            ///< packed XYZ 4:4:4, 36 bpp,
                                                                  ///< (msb) 12X, 12Y, 12Z (lsb),
                                                                  ///< the 2-byte value for each
                                                                  ///< X/Y/Z is stored as
                                                                  ///< little-endian, the 4 lower
                                                                  ///< bits are set to 0

  {stringify(AV_PIX_FMT_XYZ12BE), AV_PIX_FMT_XYZ12BE},            ///< packed XYZ 4:4:4, 36 bpp,
                                                                  ///< (msb) 12X, 12Y, 12Z (lsb),
                                                                  ///< the 2-byte value for each
                                                                  ///< X/Y/Z is stored as
                                                                  ///< big-endian, the 4 lower
                                                                  ///< bits are set to 0

  {stringify(AV_PIX_FMT_NV16), AV_PIX_FMT_NV16},                  ///< interleaved chroma
                                                                  ///< YUV 4:2:2, 16bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y samples)

  {stringify(AV_PIX_FMT_NV20LE), AV_PIX_FMT_NV16},                ///< interleaved chroma
                                                                  ///< YUV 4:2:2, 20bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y samples), little-endian

  {stringify(AV_PIX_FMT_NV20BE), AV_PIX_FMT_NV16},                ///< interleaved chroma
                                                                  ///< YUV 4:2:2, 20bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y samples), big-endian


  {stringify(AV_PIX_FMT_RGBA64BE), AV_PIX_FMT_RGBA64BE},          ///< packed RGBA 16:16:16:16,
                                                                  ///< 64bpp, 16R, 16G, 16B, 16A,
                                                                  ///< the 2-byte value for each
                                                                  ///< R/G/B/A component is stored
                                                                  ///< as big-endian

  {stringify(AV_PIX_FMT_RGBA64LE), AV_PIX_FMT_RGBA64LE},          ///< packed RGBA 16:16:16:16,
                                                                  ///< 64bpp, 16R, 16G, 16B, 16A,
                                                                  ///< the 2-byte value for each
                                                                  ///< R/G/B/A component is stored
                                                                  ///< as little-endian

  {stringify(AV_PIX_FMT_BGRA64BE), AV_PIX_FMT_BGRA64BE},          ///< packed RGBA 16:16:16:16,
                                                                  ///< 64bpp, 16B, 16G, 16R, 16A,
                                                                  ///< the 2-byte value for each
                                                                  ///< R/G/B/A component is stored
                                                                  ///< as big-endian

  {stringify(AV_PIX_FMT_BGRA64LE), AV_PIX_FMT_BGRA64LE},          ///< packed RGBA 16:16:16:16,
                                                                  ///< 64bpp, 16B, 16G, 16R, 16A,
                                                                  ///< the 2-byte value for each
                                                                  ///< R/G/B/A component is stored
                                                                  ///< as little-endian


  {stringify(AV_PIX_FMT_YVYU422), AV_PIX_FMT_YVYU422},            ///< packed YUV 4:2:2, 16bpp, Y0
                                                                  ///< Cr Y1 Cb


  {stringify(AV_PIX_FMT_YA16BE), AV_PIX_FMT_YA16BE},              ///< 16 bits gray, 16 bits alpha
                                                                  ///< (big-endian)

  {stringify(AV_PIX_FMT_YA16LE), AV_PIX_FMT_YA16LE},              ///< 16 bits gray, 16 bits alpha
                                                                  ///< (little-endian)


  {stringify(AV_PIX_FMT_GBRAP), AV_PIX_FMT_GBRAP},                ///< planar GBRA 4:4:4:4 32bpp

  {stringify(AV_PIX_FMT_GBRAP16BE), AV_PIX_FMT_GBRAP16BE},        ///< planar GBRA 4:4:4:4 64bpp,
                                                                  ///< big-endian

  {stringify(AV_PIX_FMT_GBRAP16LE), AV_PIX_FMT_GBRAP16LE},        ///< planar GBRA 4:4:4:4 64bpp,
                                                                  ///< little-endian

  /**
   *  HW acceleration through QSV, data[3] contains a pointer to the
   *  mfxFrameSurface1 structure.
   */
  {stringify(AV_PIX_FMT_QSV), AV_PIX_FMT_QSV},
  /**
   * HW acceleration though MMAL, data[3] contains a pointer to the
   * MMAL_BUFFER_HEADER_T structure.
   */
  {stringify(AV_PIX_FMT_MMAL), AV_PIX_FMT_MMAL},

  {stringify(AV_PIX_FMT_D3D11VA_VLD), AV_PIX_FMT_D3D11VA_VLD},    ///< HW decoding through
                                                                  ///< Direct3D11 via old API,
                                                                  ///< Picture.data[3] contains a
                                                                  ///< ID3D11VideoDecoderOutput-
                                                                  ///< View pointer

  /**
   * HW acceleration through CUDA. data[i] contain CUdeviceptr pointers
   * exactly as for system memory frames.
   */
  {stringify(AV_PIX_FMT_CUDA), AV_PIX_FMT_CUDA},

  {stringify(AV_PIX_FMT_0RGB), AV_PIX_FMT_0RGB},                  ///< packed RGB 8:8:8, 32bpp,
                                                                  ///< XRGBXRGB...
                                                                  ///< X=unused/undefined

  {stringify(AV_PIX_FMT_RGB0), AV_PIX_FMT_RGB0},                  ///< packed RGB 8:8:8, 32bpp,
                                                                  ///< RGBXRGBX...
                                                                  ///< X=unused/undefined

  {stringify(AV_PIX_FMT_0BGR), AV_PIX_FMT_0BGR},                  ///< packed BGR 8:8:8, 32bpp,
                                                                  ///< XBGRXBGR...
                                                                  ///< X=unused/undefined

  {stringify(AV_PIX_FMT_BGR0), AV_PIX_FMT_BGR0},                  ///< packed BGR 8:8:8, 32bpp,
                                                                  ///< BGRXBGRX...
                                                                  ///< X=unused/undefined


  {stringify(AV_PIX_FMT_YUV420P12BE), AV_PIX_FMT_YUV420P12BE},    ///< planar YUV 4:2:0,18bpp,
                                                                  ///< (1 Cr & Cb sample per 2x2
                                                                  ///< Y samples), big-endian

  {stringify(AV_PIX_FMT_YUV420P12LE), AV_PIX_FMT_YUV420P12LE},    ///< planar YUV 4:2:0,18bpp,
                                                                  ///< (1 Cr & Cb sample per 2x2
                                                                  ///< Y samples), little-endian

  {stringify(AV_PIX_FMT_YUV420P14BE), AV_PIX_FMT_YUV420P14BE},    ///< planar YUV 4:2:0,21bpp,
                                                                  ///< (1 Cr & Cb sample per 2x2
                                                                  ///< Y samples), big-endian

  {stringify(AV_PIX_FMT_YUV420P14LE), AV_PIX_FMT_YUV420P14LE},    ///< planar YUV 4:2:0,21bpp,
                                                                  ///< (1 Cr & Cb sample per 2x2
                                                                  ///< Y samples), little-endian

  {stringify(AV_PIX_FMT_YUV422P12BE), AV_PIX_FMT_YUV422P12BE},    ///< planar YUV 4:2:2,24bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y samples), big-endian

  {stringify(AV_PIX_FMT_YUV422P12LE), AV_PIX_FMT_YUV422P12LE},    ///< planar YUV 4:2:2,24bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y samples), little-endian

  {stringify(AV_PIX_FMT_YUV422P14BE), AV_PIX_FMT_YUV422P14BE},    ///< planar YUV 4:2:2,28bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y samples), big-endian

  {stringify(AV_PIX_FMT_YUV422P14LE), AV_PIX_FMT_YUV422P14LE},    ///< planar YUV 4:2:2,28bpp,
                                                                  ///< (1 Cr & Cb sample per 2x1
                                                                  ///< Y samples), little-endian

  {stringify(AV_PIX_FMT_YUV444P12BE), AV_PIX_FMT_YUV444P12BE},    ///< planar YUV 4:4:4,36bpp,
                                                                  ///< (1 Cr & Cb sample per 1x1
                                                                  ///< Y samples), big-endian

  {stringify(AV_PIX_FMT_YUV444P12LE), AV_PIX_FMT_YUV444P12LE},    ///< planar YUV 4:4:4,36bpp,
                                                                  ///< (1 Cr & Cb sample per 1x1
                                                                  ///< Y samples), little-endian

  {stringify(AV_PIX_FMT_YUV444P14BE), AV_PIX_FMT_YUV444P14BE},    ///< planar YUV 4:4:4,42bpp,
                                                                  ///< (1 Cr & Cb sample per 1x1
                                                                  ///< Y samples), big-endian

  {stringify(AV_PIX_FMT_YUV444P14LE), AV_PIX_FMT_YUV444P14LE},    ///< planar YUV 4:4:4,42bpp,
                                                                  ///< (1 Cr & Cb sample per 1x1
                                                                  ///< Y samples), little-endian


  {stringify(AV_PIX_FMT_GBRP12BE), AV_PIX_FMT_GBRP12BE},          ///< planar GBR 4:4:4 36bpp,
                                                                  ///< big-endian

  {stringify(AV_PIX_FMT_GBRP12LE), AV_PIX_FMT_GBRP12LE},          ///< planar GBR 4:4:4 36bpp,
                                                                  ///< little-endian

  {stringify(AV_PIX_FMT_GBRP14BE), AV_PIX_FMT_GBRP14BE},          ///< planar GBR 4:4:4 42bpp,
                                                                  ///< big-endian

  {stringify(AV_PIX_FMT_GBRP14LE), AV_PIX_FMT_GBRP14LE},          ///< planar GBR 4:4:4 42bpp,
                                                                  ///< little-endian

  {stringify(AV_PIX_FMT_YUVJ411P), AV_PIX_FMT_YUVJ411P},          ///< planar YUV 4:1:1, 12bpp,
                                                                  ///< (1 Cr & Cb sample per 4x1
                                                                  ///< Y samples) full scale
                                                                  ///< (JPEG), deprecated in favor
                                                                  ///< of AV_PIX_FMT_YUV411P and
                                                                  ///< setting color_range


  {stringify(AV_PIX_FMT_BAYER_BGGR8), AV_PIX_FMT_BAYER_BGGR8},        ///< bayer,
                                                                      ///< BGBG..(odd line),
                                                                      ///< GRGR..(even line),
                                                                      ///< 8-bit samples

  {stringify(AV_PIX_FMT_BAYER_RGGB8), AV_PIX_FMT_BAYER_RGGB8},        ///< bayer,
                                                                      ///< RGRG..(odd line),
                                                                      ///< GBGB..(even line),
                                                                      ///< 8-bit samples

  {stringify(AV_PIX_FMT_BAYER_GBRG8), AV_PIX_FMT_BAYER_GBRG8},        ///< bayer,
                                                                      ///< GBGB..(odd line),
                                                                      ///< RGRG..(even line),
                                                                      ///< 8-bit samples

  {stringify(AV_PIX_FMT_BAYER_GRBG8), AV_PIX_FMT_BAYER_GRBG8},        ///< bayer,
                                                                      ///< GRGR..(odd line),
                                                                      ///< BGBG..(even line),
                                                                      ///< 8-bit samples

  {stringify(AV_PIX_FMT_BAYER_BGGR16LE), AV_PIX_FMT_BAYER_BGGR16LE},  ///< bayer,
                                                                      ///< BGBG..(odd line),
                                                                      ///< GRGR..(even line),
                                                                      ///< 16-bit samples,
                                                                      ///< little-endian

  {stringify(AV_PIX_FMT_BAYER_BGGR16BE), AV_PIX_FMT_BAYER_BGGR16BE},  ///< bayer,
                                                                      ///< BGBG..(odd line),
                                                                      ///< GRGR..(even line),
                                                                      ///< 16-bit samples,
                                                                      ///< big-endian

  {stringify(AV_PIX_FMT_BAYER_RGGB16LE), AV_PIX_FMT_BAYER_RGGB16LE},  ///< bayer,
                                                                      ///< RGRG..(odd line),
                                                                      ///< GBGB..(even line),
                                                                      ///< 16-bit samples,
                                                                      ///< little-endian

  {stringify(AV_PIX_FMT_BAYER_RGGB16BE), AV_PIX_FMT_BAYER_RGGB16BE},  ///< bayer,
                                                                      ///< RGRG..(odd line),
                                                                      ///< GBGB..(even line),
                                                                      ///< 16-bit samples,
                                                                      ///< big-endian

  {stringify(AV_PIX_FMT_BAYER_GBRG16LE), AV_PIX_FMT_BAYER_GBRG16LE},  ///< bayer,
                                                                      ///< GBGB..(odd line),
                                                                      ///< RGRG..(even line),
                                                                      ///< 16-bit samples,
                                                                      ///< little-endian

  {stringify(AV_PIX_FMT_BAYER_GBRG16BE), AV_PIX_FMT_BAYER_GBRG16BE},  ///< bayer,
                                                                      ///< GBGB..(odd line),
                                                                      ///< RGRG..(even line),
                                                                      ///< 16-bit samples,
                                                                      ///< big-endian

  {stringify(AV_PIX_FMT_BAYER_GRBG16LE), AV_PIX_FMT_BAYER_GRBG16LE},  ///< bayer,
                                                                      ///< GRGR..(odd line),
                                                                      ///< BGBG..(even line),
                                                                      ///< 16-bit samples,
                                                                      ///< little-endian

  {stringify(AV_PIX_FMT_BAYER_GRBG16BE), AV_PIX_FMT_BAYER_GRBG16BE},  ///< bayer,
                                                                      ///< GRGR..(odd line),
                                                                      ///< BGBG..(even line),
                                                                      ///< 16-bit samples,
                                                                      ///< big-endian


  {stringify(AV_PIX_FMT_XVMC), AV_PIX_FMT_XVMC},                      ///< XVideo Motion
                                                                      ///< Acceleration via common
                                                                      ///< packet passing


  {stringify(AV_PIX_FMT_YUV440P10LE), AV_PIX_FMT_YUV440P10LE},        ///< planar YUV 4:4:0,20bpp,
                                                                      ///< (1 Cr & Cb sample per
                                                                      ///< 1x2 Y samples),
                                                                      ///< little-endian

  {stringify(AV_PIX_FMT_YUV440P10BE), AV_PIX_FMT_YUV440P10BE},        ///< planar YUV 4:4:0,20bpp,
                                                                      ///< (1 Cr & Cb sample per
                                                                      ///< 1x2 Y samples),
                                                                      ///< big-endian

  {stringify(AV_PIX_FMT_YUV440P12LE), AV_PIX_FMT_YUV440P12LE},        ///< planar YUV 4:4:0,24bpp,
                                                                      ///< (1 Cr & Cb sample per
                                                                      ///< 1x2 Y samples),
                                                                      ///< little-endian

  {stringify(AV_PIX_FMT_YUV440P12BE), AV_PIX_FMT_YUV440P12BE},        ///< planar YUV 4:4:0,24bpp,
                                                                      ///< (1 Cr & Cb sample per
                                                                      ///< 1x2 Y samples),
                                                                      ///< big-endian

  {stringify(AV_PIX_FMT_AYUV64LE), AV_PIX_FMT_AYUV64LE},              ///< packed AYUV 4:4:4,64bpp
                                                                      ///< (1 Cr & Cb sample per
                                                                      ///< 1x1 Y & A samples),
                                                                      ///< little-endian

  {stringify(AV_PIX_FMT_AYUV64BE), AV_PIX_FMT_AYUV64BE},              ///< packed AYUV 4:4:4,64bpp
                                                                      ///< (1 Cr & Cb sample per
                                                                      ///< 1x1 Y & A samples),
                                                                      ///< big-endian


  {stringify(AV_PIX_FMT_VIDEOTOOLBOX), AV_PIX_FMT_VIDEOTOOLBOX},      ///< hardware decoding
                                                                      ///< through Videotoolbox


  {stringify(AV_PIX_FMT_P010LE), AV_PIX_FMT_P010LE},                  ///< like NV12, with 10bpp
                                                                      ///< per component, data in
                                                                      ///< the high bits, zeros in
                                                                      ///< the low bits,
                                                                      ///< little-endian

  {stringify(AV_PIX_FMT_P010BE), AV_PIX_FMT_P010BE},                  ///< like NV12, with 10bpp
                                                                      ///< per component, data in
                                                                      ///< the high bits, zeros in
                                                                      ///< the low bits,
                                                                      ///< big-endian


  {stringify(AV_PIX_FMT_GBRAP12BE), AV_PIX_FMT_GBRAP12BE},            ///< planar GBR 4:4:4:4
                                                                      ///< 48bpp, big-endian

  {stringify(AV_PIX_FMT_GBRAP12LE), AV_PIX_FMT_GBRAP12LE},            ///< planar GBR 4:4:4:4
                                                                      ///< 48bpp, little-endian


  {stringify(AV_PIX_FMT_GBRAP10BE), AV_PIX_FMT_GBRAP10BE},            ///< planar GBR 4:4:4:4
                                                                      ///< 40bpp, big-endian

  {stringify(AV_PIX_FMT_GBRAP10LE), AV_PIX_FMT_GBRAP10LE},            ///< planar GBR 4:4:4:4
                                                                      ///< 40bpp, little-endian


  {stringify(AV_PIX_FMT_MEDIACODEC), AV_PIX_FMT_MEDIACODEC},          ///< hardware decoding
                                                                      ///< through MediaCodec


  {stringify(AV_PIX_FMT_GRAY12BE), AV_PIX_FMT_GRAY12BE},              ///< Y, 12bpp, big-endian

  {stringify(AV_PIX_FMT_GRAY12LE), AV_PIX_FMT_GRAY12LE},              ///< Y, 12bpp, little-endian

  {stringify(AV_PIX_FMT_GRAY10BE), AV_PIX_FMT_GRAY10BE},              ///< Y, 10bpp, big-endian

  {stringify(AV_PIX_FMT_GRAY10LE), AV_PIX_FMT_GRAY10LE},              ///< Y, 10bpp, little-endian


  {stringify(AV_PIX_FMT_P016LE), AV_PIX_FMT_P016LE},                  ///< like NV12, with 16bpp
                                                                      ///< per component,
                                                                      ///< little-endian

  {stringify(AV_PIX_FMT_P016BE), AV_PIX_FMT_P016BE},                  ///< like NV12, with 16bpp
                                                                      ///< per component,
                                                                      ///< big-endian

  /**
   * Hardware surfaces for Direct3D11.
   *
   * This is preferred over the legacy AV_PIX_FMT_D3D11VA_VLD. The new D3D11
   * hwaccel API and filtering support AV_PIX_FMT_D3D11 only.
   *
   * data[0] contains a ID3D11Texture2D pointer, and data[1] contains the
   * texture array index of the frame as intptr_t if the ID3D11Texture2D is
   * an array texture (or always 0 if it's a normal texture).
   */
  {stringify(AV_PIX_FMT_D3D11), AV_PIX_FMT_D3D11},

  {stringify(AV_PIX_FMT_GRAY9BE), AV_PIX_FMT_GRAY9BE},                ///< Y, 9bpp, big-endian

  {stringify(AV_PIX_FMT_GRAY9LE), AV_PIX_FMT_GRAY9LE},                ///< Y, 9bpp, little-endian


  {stringify(AV_PIX_FMT_GBRPF32BE), AV_PIX_FMT_GBRPF32BE},            ///< IEEE-754 single
                                                                      ///< precision planar
                                                                      ///< GBR 4:4:4, 96bpp,
                                                                      ///< big-endian

  {stringify(AV_PIX_FMT_GBRPF32LE), AV_PIX_FMT_GBRPF32LE},            ///< IEEE-754 single
                                                                      ///< precision planar GBR
                                                                      ///< 4:4:4, 96bpp,
                                                                      ///< little-endian

  {stringify(AV_PIX_FMT_GBRAPF32BE), AV_PIX_FMT_GBRAPF32BE},          ///< IEEE-754 single
                                                                      ///< precision planar GBRA
                                                                      ///< 4:4:4:4, 128bpp,
                                                                      ///< big-endian

  {stringify(AV_PIX_FMT_GBRAPF32LE), AV_PIX_FMT_GBRAPF32LE},          ///< IEEE-754 single
                                                                      ///< precision planar GBRA
                                                                      ///< 4:4:4:4, 128bpp,
                                                                      ///< little-endian


  /**
   * DRM-managed buffers exposed through PRIME buffer sharing.
   *
   * data[0] points to an AVDRMFrameDescriptor.
   */
  {stringify(AV_PIX_FMT_DRM_PRIME), AV_PIX_FMT_DRM_PRIME},
  /**
   * Hardware surfaces for OpenCL.
   *
   * data[i] contain 2D image objects (typed in C as cl_mem, used
   * in OpenCL as image2d_t) for each plane of the surface.
   */
  {stringify(AV_PIX_FMT_OPENCL), AV_PIX_FMT_OPENCL},

  {stringify(AV_PIX_FMT_GRAY14BE), AV_PIX_FMT_GRAY14BE},              ///< Y, 14bpp, big-endian

  {stringify(AV_PIX_FMT_GRAY14LE), AV_PIX_FMT_GRAY14LE},              ///< Y, 14bpp, little-endian


  {stringify(AV_PIX_FMT_GRAYF32BE), AV_PIX_FMT_GRAYF32BE},            ///< IEEE-754 single
                                                                      ///< precision Y, 32bpp,
                                                                      ///< big-endian

  {stringify(AV_PIX_FMT_GRAYF32LE), AV_PIX_FMT_GRAYF32LE},            ///< IEEE-754 single
                                                                      ///< precision Y, 32bpp,
                                                                      ///< little-endian


  {stringify(AV_PIX_FMT_YUVA422P12BE), AV_PIX_FMT_YUVA422P12BE},      ///< planar YUV 4:2:2,24bpp,
                                                                      ///< (1 Cr & Cb sample per
                                                                      ///< 2x1 Y samples),
                                                                      ///< 12b alpha, big-endian

  {stringify(AV_PIX_FMT_YUVA422P12LE), AV_PIX_FMT_YUVA422P12LE},      ///< planar YUV 4:2:2,24bpp,
                                                                      ///< (1 Cr & Cb sample per
                                                                      ///< 2x1 Y samples),
                                                                      ///< 12b alpha,
                                                                      ///< little-endian

  {stringify(AV_PIX_FMT_YUVA444P12BE), AV_PIX_FMT_YUVA444P12BE},      ///< planar YUV 4:4:4,36bpp,
                                                                      ///< (1 Cr & Cb sample per
                                                                      ///< 1x1 Y samples),
                                                                      ///< 12b alpha, big-endian

  {stringify(AV_PIX_FMT_YUVA444P12LE), AV_PIX_FMT_YUVA444P12LE},      ///< planar YUV 4:4:4,36bpp,
                                                                      ///< (1 Cr & Cb sample per
                                                                      ///< 1x1 Y samples), 12b
                                                                      ///< alpha, little-endian


  {stringify(AV_PIX_FMT_NV24), AV_PIX_FMT_NV24},                      ///< planar YUV 4:4:4,
                                                                      ///< 24bpp, 1 plane for Y
                                                                      ///< and 1 plane for the UV
                                                                      ///< components, which are
                                                                      ///< interleaved (first byte
                                                                      ///< U and the following
                                                                      ///< byte V)

  {stringify(AV_PIX_FMT_NV42), AV_PIX_FMT_NV42},                      ///< as above, but U and V
                                                                      ///< bytes are swapped

  {stringify(AV_PIX_FMT_NB), AV_PIX_FMT_NB},                          ///< number of pixel
                                                                      ///< formats, DO NOT USE
                                                                      ///< THIS if you want to
                                                                      ///< link with shared libav*
                                                                      ///< because the number of
                                                                      ///< formats might differ
                                                                      ///< between versions
};

/// @brief Get AVPixelFormat from string. This string should correspond to the AVPixelFormat name.
/// The name can either be given with or without the 'AV_PIX_FMT_' prefix.
/// @param str AVPixelFormat name
/// @return Pixel format enum corresponding to a given name
inline AVPixelFormat get_av_pixel_format_from_string(const std::string & str)
{
  std::string upperCaseStr = str;
  std::transform(upperCaseStr.begin(), upperCaseStr.end(), upperCaseStr.begin(), ::toupper);

  std::string fullFmtStr;
  if (upperCaseStr.rfind("AV_PIX_FMT_", 0) == std::string::npos) {
    // passed string does not start with 'AV_PIX_FMT_'
    fullFmtStr = "AV_PIX_FMT_" + upperCaseStr;
  } else {
    fullFmtStr = upperCaseStr;
  }

  return STR_2_AVPIXFMT.find(fullFmtStr)->second;
}


/// @brief Get ROS PixelFormat from AVPixelFormat.
/// @param avPixelFormat AVPixelFormat
/// @return String specifying the ROS pixel format.
inline std::string get_ros_pixel_format_from_av_format(const AVPixelFormat & avPixelFormat)
{
  std::string ros_format = "";

  switch (avPixelFormat) {
    default:
      {
        ros_format = usb_cam::constants::UNKNOWN;
      }
      break;

    case AVPixelFormat::AV_PIX_FMT_RGB24:
      {
        ros_format = usb_cam::constants::RGB8;
      }
      break;

    case AVPixelFormat::AV_PIX_FMT_RGBA:
      {
        ros_format = usb_cam::constants::RGBA8;
      }
      break;

    case AVPixelFormat::AV_PIX_FMT_BGR24:
      {
        ros_format = usb_cam::constants::BGR8;
      }
      break;

    case AVPixelFormat::AV_PIX_FMT_BGRA:
      {
        ros_format = usb_cam::constants::BGRA8;
      }
      break;

    case AVPixelFormat::AV_PIX_FMT_GRAY8:
      {
        ros_format = usb_cam::constants::MONO8;
      }
      break;

    case AVPixelFormat::AV_PIX_FMT_GRAY16BE:
      {
        ros_format = usb_cam::constants::MONO16;
      }
      break;

    case AVPixelFormat::AV_PIX_FMT_YUV422P:
      {
        ros_format = usb_cam::constants::YUV422;
      }
      break;

    case AVPixelFormat::AV_PIX_FMT_YUV420P:
      {
        ros_format = usb_cam::constants::NV21;
      }
      break;

    case AVPixelFormat::AV_PIX_FMT_YUV444P:
      {
        ros_format = usb_cam::constants::NV24;
      }
      break;
  }

  return ros_format;
}


/// @brief Overload to support av pixel formats being passed as strings
/// @param avPixelFormat AvPixelFormat as string
/// @return String specifying the ROS pixel format.
inline std::string get_ros_pixel_format_from_av_format(const std::string avPixelFormatStr)
{
  auto fmt = get_av_pixel_format_from_string(avPixelFormatStr);
  return get_ros_pixel_format_from_av_format(fmt);
}


/// @brief Get the number of channels from AVPixelFormat.
/// @param avPixelFormat AVPixelFormat
/// @return Number of channels as uint8
inline uint8_t get_channels_from_av_format(const AVPixelFormat & avPixelFormat)
{
  uint8_t channels = 1;

  switch (avPixelFormat) {
    default:
    case AVPixelFormat::AV_PIX_FMT_GRAY8:
    case AVPixelFormat::AV_PIX_FMT_GRAY16BE:
      {
        channels = 1;
      }
      break;

    case AVPixelFormat::AV_PIX_FMT_YUV422P:
    case AVPixelFormat::AV_PIX_FMT_YUV420P:
    case AVPixelFormat::AV_PIX_FMT_YUV444P:
      {
        channels = 2;
      }
      break;

    case AVPixelFormat::AV_PIX_FMT_RGB24:
    case AVPixelFormat::AV_PIX_FMT_BGR24:
      {
        channels = 3;
      }
      break;

    case AVPixelFormat::AV_PIX_FMT_RGBA:
    case AVPixelFormat::AV_PIX_FMT_BGRA:
      {
        channels = 4;
      }
      break;
  }

  return channels;
}


/// @brief Overload of function below to support av pixel formats as strings
/// @param avPixelFormatStr AvPixelFormat as string
/// @return Number of channels as uint8_t
inline uint8_t get_channels_from_av_format(const std::string & avPixelFormatStr)
{
  auto fmt = get_av_pixel_format_from_string(avPixelFormatStr);
  return get_channels_from_av_format(fmt);
}


/// @brief Get the pixel bit depth from AVPixelFormat.
/// @param avPixelFormat AVPixelFormat
/// @return Bit depth as uint8
inline uint8_t get_bit_depth_from_av_format(const AVPixelFormat & avPixelFormat)
{
  uint8_t bit_depth = 8;

  switch (avPixelFormat) {
    default:
    case AVPixelFormat::AV_PIX_FMT_GRAY8:
    case AVPixelFormat::AV_PIX_FMT_RGB24:
    case AVPixelFormat::AV_PIX_FMT_BGR24:
    case AVPixelFormat::AV_PIX_FMT_RGBA:
    case AVPixelFormat::AV_PIX_FMT_BGRA:
    case AVPixelFormat::AV_PIX_FMT_YUV422P:
    case AVPixelFormat::AV_PIX_FMT_YUV420P:
    case AVPixelFormat::AV_PIX_FMT_YUV444P:
      {
        bit_depth = 8;
      }
      break;

    case AVPixelFormat::AV_PIX_FMT_GRAY16BE:
      {
        bit_depth = 16;
      }
      break;
  }

  return bit_depth;
}


/// @brief Overload of function below to support passing av pixel formats as strings
/// @param avPixelFormatStr AVPixelFormat as string
/// @return Bit depth as uint8
inline uint8_t get_bit_depth_from_av_format(const std::string & avPixelFormatStr)
{
  auto fmt = get_av_pixel_format_from_string(avPixelFormatStr);
  return get_bit_depth_from_av_format(fmt);
}

}  // namespace formats
}  // namespace usb_cam

#endif  // USB_CAM__FORMATS__AV_PIXEL_FORMAT_HELPER_HPP_
