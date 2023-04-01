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


#ifndef USB_CAM__FORMATS__UTILS_HPP_
#define USB_CAM__FORMATS__UTILS_HPP_

#include "usb_cam/utils.hpp"

namespace usb_cam
{
namespace formats
{


/** Clip a value to the range 0<val<255. For speed this is done using an
 * array, so can only cope with numbers in the range -128<=val<=383.
 */
inline unsigned char CLIPVALUE(const int & val)
{
  // Old method (if)
  /*   val = val < 0 ? 0 : val; */
  /*   return val > 255 ? 255 : val; */

  try {
    // New method array
    return usb_cam::constants::uchar_clipping_table.at(
      val + usb_cam::constants::clipping_table_offset);
  } catch (std::out_of_range const &) {
    // fall back to old method
    unsigned char clipped_val = val < 0 ? 0 : static_cast<unsigned char>(val);
    return val > 255 ? 255 : clipped_val;
  }
}

/// @brief Conversion from YUV to RGB.
///
/// The normal conversion matrix is due to Julien (surname unknown):
///
/// [ R ]   [  1.0   0.0     1.403 ] [ Y ]
/// [ G ] = [  1.0  -0.344  -0.714 ] [ U ]
/// [ B ]   [  1.0   1.770   0.0   ] [ V ]
///
/// and the firewire one is similar:
///
/// [ R ]   [  1.0   0.0     0.700 ] [ Y ]
/// [ G ] = [  1.0  -0.198  -0.291 ] [ U ]
/// [ B ]   [  1.0   1.015   0.0   ] [ V ]
///
/// Corrected by BJT (coriander's transforms RGB->YUV and YUV->RGB
///                   do not get you back to the same RGB!)
/// [ R ]   [  1.0   0.0     1.136 ] [ Y ]
/// [ G ] = [  1.0  -0.396  -0.578 ] [ U ]
/// [ B ]   [  1.0   2.041   0.002 ] [ V ]
///
inline void YUV2RGB(
  const unsigned char & y, const unsigned char & u, const unsigned char & v,
  unsigned char * r, unsigned char * g, unsigned char * b)
{
  const int y2 = static_cast<int>(y);
  const int u2 = static_cast<int>(u - 128);
  const int v2 = static_cast<int>(v - 128);

  // This is the normal YUV conversion, but
  // appears to be incorrect for the firewire cameras
  //   int r2 = y2 + ( (v2*91947) >> 16);
  //   int g2 = y2 - ( ((u2*22544) + (v2*46793)) >> 16 );
  //   int b2 = y2 + ( (u2*115999) >> 16);
  // This is an adjusted version (UV spread out a bit)
  int r2 = y2 + ((v2 * 37221) >> 15);
  int g2 = y2 - (((u2 * 12975) + (v2 * 18949)) >> 15);
  int b2 = y2 + ((u2 * 66883) >> 15);

  // Cap the values.
  *r = CLIPVALUE(r2);
  *g = CLIPVALUE(g2);
  *b = CLIPVALUE(b2);
}

}  // namespace formats
}  // namespace usb_cam

#endif  // USB_CAM__FORMATS__UTILS_HPP_
