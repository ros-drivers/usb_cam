#include "usb_cam/util.h"

using namespace usb_cam;

time_t get_epoch_time_shift()
{
    struct timeval epoch_time;
    struct timespec monotonic_time;

    gettimeofday(&epoch_time, NULL);
    clock_gettime(CLOCK_MONOTONIC, &monotonic_time);

    const int64_t uptime_ms =
            monotonic_time.tv_sec * 1000 + static_cast<int64_t>(
                std::round(monotonic_time.tv_nsec / 1000000.0));
    const int64_t epoch_ms =
            epoch_time.tv_sec * 1000 + static_cast<int64_t>(
                std::round(epoch_time.tv_usec / 1000.0));

    return static_cast<time_t>((epoch_ms - uptime_ms) / 1000);
}

int util::xioctl(int fd, int request, void * arg)
{
    int r;
    do
    {
        r = ioctl(fd, request, arg);
        continue;
    } while (-1 == r && EINTR == errno);

    return r;
}

unsigned char util::CLIPVALUE(const int & val)
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

time_t util::get_epoch_time_shift()
{
    struct timeval epoch_time;
    struct timespec monotonic_time;
    gettimeofday(&epoch_time, NULL);
    clock_gettime(CLOCK_MONOTONIC, &monotonic_time);
    const int64_t uptime_ms =
        monotonic_time.tv_sec * 1000 + static_cast<int64_t>(
            std::round(monotonic_time.tv_nsec / 1000000.0));
    const int64_t epoch_ms =
        epoch_time.tv_sec * 1000 + static_cast<int64_t>(
            std::round(epoch_time.tv_usec / 1000.0));
    return static_cast<time_t>((epoch_ms - uptime_ms) / 1000);
}
