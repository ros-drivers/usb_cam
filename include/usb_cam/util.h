#ifndef USB_CAM_UTIL_H
#define USB_CAM_UTIL_H

#include "usb_cam/types.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace usb_cam
{
namespace util
{

/// @brief Get epoch time shift
/// @details Run this at start of process to calculate epoch time shift
/// @ref https://stackoverflow.com/questions/10266451/where-does-v4l2-buffer-timestamp-value-starts-counting
time_t get_epoch_time_shift_us();

/// @brief Calculate image timestamp from buffer time and epoch time shift.
/// In this, the buffer time is first converted into microseconds before the epoch time shift,
/// which is to be given in microseconds is added to it. Afterwards it is split into seconds
/// and nanoseconds for the image timestamp.
timespec calc_img_timestamp(const timeval & buffer_time, const time_t & epoch_time_shift_us);

int xioctl(int fd, int request, void * arg);

/** Clip a value to the range 0<val<255. For speed this is done using an
 * array, so can only cope with numbers in the range -128<=val<=383.
 */
unsigned char CLIPVALUE(const int & val);

}  // namespace util
}  // namespace usb_cam

#endif
