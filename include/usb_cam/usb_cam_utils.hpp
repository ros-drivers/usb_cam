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
//    * Neither the name of the Robert Bosch, LLC nor the names of its
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

#ifndef USB_CAM__USB_CAM_UTILS_HPP_
#define USB_CAM__USB_CAM_UTILS_HPP_

#include <sys/ioctl.h>

#include <cstring>
#include <sstream>


namespace usb_cam
{

void monotonicToRealTime(const timespec & monotonic_time, timespec & real_time)
{
  struct timespec real_sample1, real_sample2, monotonic_sample;

  // TODO(lucasw) Disable interrupts here?
  // otherwise what if there is a delay/interruption between sampling the times?
  clock_gettime(CLOCK_REALTIME, &real_sample1);
  clock_gettime(CLOCK_MONOTONIC, &monotonic_sample);
  clock_gettime(CLOCK_REALTIME, &real_sample2);

  timespec time_diff;
  time_diff.tv_sec = real_sample2.tv_sec - monotonic_sample.tv_sec;
  time_diff.tv_nsec = real_sample2.tv_nsec - monotonic_sample.tv_nsec;

  // This isn't available outside of the kernel
  // real_time = timespec_add(monotonic_time, time_diff);

  const int64_t NSEC_PER_SEC = 1000000000;
  real_time.tv_sec = monotonic_time.tv_sec + time_diff.tv_sec;
  real_time.tv_nsec = monotonic_time.tv_nsec + time_diff.tv_nsec;
  if (real_time.tv_nsec >= NSEC_PER_SEC) {
    ++real_time.tv_sec;
    real_time.tv_nsec -= NSEC_PER_SEC;
  } else if (real_time.tv_nsec < 0) {
    --real_time.tv_sec;
    real_time.tv_nsec += NSEC_PER_SEC;
  }
}

inline int xioctl(int fd, unsigned long request, void * arg)
{
  int r;

  do {
    r = ioctl(fd, request, arg);
    continue;
  } while (-1 == r && EINTR == errno);

  return r;
}

const unsigned char uchar_clipping_table[] = {
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,  // -128 - -121
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,  // -120 - -113
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,  // -112 - -105
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,  // -104 -  -97
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,  //  -96 -  -89
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,  //  -88 -  -81
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,  //  -80 -  -73
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,  //  -72 -  -65
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,  //  -64 -  -57
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,  //  -56 -  -49
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,  //  -48 -  -41
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,  //  -40 -  -33
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,  //  -32 -  -25
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,  //  -24 -  -17
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,  //  -16 -   -9
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,  //   -8 -   -1
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

/** Clip a value to the range 0<val<255. For speed this is done using an
 * array, so can only cope with numbers in the range -128<val<383.
 */
inline unsigned char CLIPVALUE(int val)
{
  // Old method (if)
  /*   val = val < 0 ? 0 : val; */
  /*   return val > 255 ? 255 : val; */

  // New method (array)
  return uchar_clipping_table[val + clipping_table_offset];
}

/**
 * Conversion from YUV to RGB.
 * The normal conversion matrix is due to Julien (surname unknown):
 *
 * [ R ]   [  1.0   0.0     1.403 ] [ Y ]
 * [ G ] = [  1.0  -0.344  -0.714 ] [ U ]
 * [ B ]   [  1.0   1.770   0.0   ] [ V ]
 *
 * and the firewire one is similar:
 *
 * [ R ]   [  1.0   0.0     0.700 ] [ Y ]
 * [ G ] = [  1.0  -0.198  -0.291 ] [ U ]
 * [ B ]   [  1.0   1.015   0.0   ] [ V ]
 *
 * Corrected by BJT (coriander's transforms RGB->YUV and YUV->RGB
 *                   do not get you back to the same RGB!)
 * [ R ]   [  1.0   0.0     1.136 ] [ Y ]
 * [ G ] = [  1.0  -0.396  -0.578 ] [ U ]
 * [ B ]   [  1.0   2.041   0.002 ] [ V ]
 *
 */
inline bool YUV2RGB(
  const unsigned char y, const unsigned char u, const unsigned char v,
  unsigned char * r, unsigned char * g, unsigned char * b)
{
  const int y2 = static_cast<int>(y);
  const int u2 = static_cast<int>(u - 128);
  const int v2 = static_cast<int>(v - 128);
  // std::cerr << "YUV=("<<y2<<","<<u2<<","<<v2<<")"<<std::endl;

  // This is the normal YUV conversion, but
  // appears to be incorrect for the firewire cameras
  //   int r2 = y2 + ( (v2*91947) >> 16);
  //   int g2 = y2 - ( ((u2*22544) + (v2*46793)) >> 16 );
  //   int b2 = y2 + ( (u2*115999) >> 16);
  // This is an adjusted version (UV spread out a bit)
  int r2 = y2 + ((v2 * 37221) >> 15);
  int g2 = y2 - (((u2 * 12975) + (v2 * 18949)) >> 15);
  int b2 = y2 + ((u2 * 66883) >> 15);
  // std::cerr << "   RGB=("<<r2<<","<<g2<<","<<b2<<")"<<std::endl;

  // Cap the values.
  *r = CLIPVALUE(r2);
  *g = CLIPVALUE(g2);
  *b = CLIPVALUE(b2);

  return true;
}

bool uyvy2rgb(char * YUV, char * RGB, int NumPixels)
{
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r, g, b;
  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6) {
    u = (unsigned char)YUV[i + 0];
    y0 = (unsigned char)YUV[i + 1];
    v = (unsigned char)YUV[i + 2];
    y1 = (unsigned char)YUV[i + 3];
    YUV2RGB(y0, u, v, &r, &g, &b);
    RGB[j + 0] = r;
    RGB[j + 1] = g;
    RGB[j + 2] = b;
    YUV2RGB(y1, u, v, &r, &g, &b);
    RGB[j + 3] = r;
    RGB[j + 4] = g;
    RGB[j + 5] = b;
  }
  return true;
}

inline bool mono102mono8(char * RAW, char * MONO, int NumPixels)
{
  int i, j;
  for (i = 0, j = 0; i < (NumPixels << 1); i += 2, j += 1) {
    // first byte is low byte, second byte is high byte; smash together and convert to 8-bit
    MONO[j] = (unsigned char)(((RAW[i + 0] >> 2) & 0x3F) | ((RAW[i + 1] << 6) & 0xC0));
  }
  return true;
}

inline bool yuyv2rgb(char * YUV, char * RGB, int NumPixels)
{
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r, g, b;

  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6) {
    y0 = (unsigned char)YUV[i + 0];
    u = (unsigned char)YUV[i + 1];
    y1 = (unsigned char)YUV[i + 2];
    v = (unsigned char)YUV[i + 3];
    YUV2RGB(y0, u, v, &r, &g, &b);
    RGB[j + 0] = r;
    RGB[j + 1] = g;
    RGB[j + 2] = b;
    YUV2RGB(y1, u, v, &r, &g, &b);
    RGB[j + 3] = r;
    RGB[j + 4] = g;
    RGB[j + 5] = b;
  }
  return true;
}

void rgb242rgb(char * YUV, char * RGB, int NumPixels)
{
  memcpy(RGB, YUV, NumPixels * 3);
}

std::string fcc2s(unsigned int val)
{
	std::string s;

	s += val & 0x7f;
	s += (val >> 8) & 0x7f;
	s += (val >> 16) & 0x7f;
	s += (val >> 24) & 0x7f;
	if (val & (1 << 31))
		s += "-BE";
	return s;
}
}  // namespace usb_cam
#endif  // USB_CAM__USB_CAM_UTILS_HPP_
