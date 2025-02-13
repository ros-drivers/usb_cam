#include <iostream>

#include "linux/videodev2.h"

#include "usb_cam/constants.hpp"
#include "usb_cam/device/capability.hpp"

namespace usb_cam
{
namespace device
{

using usb_cam::constants::DIVIDER;


void capability::print()
{
    std::cout << "Driver: " << driver << std::endl;
    std::cout << "Card: " << card << std::endl;
    std::cout << "Bus Info: " << bus_info << std::endl;
    std::cout << "Version: " << (version >> 16) << "." << ((version >> 8) & 0xFF) << "." << (version & 0xFF) << std::endl;
    std::cout << DIVIDER << std::endl;
    std::cout << "Capabilities: " << std::endl;

    if (is_media_controller_centric())
    {
        std::cout << "  - is meida controller centric" << std::endl;
    }
    else
    {
        std::cout << "  - is video node centric" << std::endl;
    }
    if (supports_video_capture())
    {
        std::cout << "  - video capture" << std::endl;
    }
    if (supports_video_capture_mplane())
    {
        std::cout << "  - video capture mplane" << std::endl;
    }
    if (supports_streaming())
    {
        std::cout << "  - streaming I/O" << std::endl;
    }
    if (supports_audio())
    {
        std::cout << "  - audio I/O" << std::endl;
    }
    if (supports_tuner())
    {
        std::cout << "  - tuner I/O" << std::endl;
    }
    if (supports_ext_pix_format())
    {
        std::cout << "  - extended pixel format" << std::endl;
    }
    if (supports_video_overlay())
    {
        std::cout << "  - video overlay" << std::endl;
    }

    std::cout << DIVIDER << std::endl;
}

bool capability::supports_video_capture()
{
    return capabilities & V4L2_CAP_VIDEO_CAPTURE;
}

bool capability::supports_video_capture_mplane()
{
    return capabilities & V4L2_CAP_STREAMING;
}

bool capability::supports_video_output()
{
    return capabilities & V4L2_CAP_VIDEO_OUTPUT;
}

bool capability::supports_video_output_mplane()
{
    return capabilities & V4L2_CAP_VIDEO_OUTPUT_MPLANE;
}

bool capability::supports_video_m2m()
{
    return capabilities & V4L2_CAP_VIDEO_M2M;
}

bool capability::supports_video_m2m_mplane()
{
    return capabilities & V4L2_CAP_VIDEO_M2M_MPLANE;
}

bool capability::supports_video_overlay()
{
    return capabilities & V4L2_CAP_VIDEO_OVERLAY;
}

bool capability::supports_vbi_capture()
{
    return capabilities & V4L2_CAP_VBI_CAPTURE;
}

bool capability::supports_vbi_output()
{
    return capabilities & V4L2_CAP_VBI_OUTPUT;
}

bool capability::supports_sliced_vbi_capture()
{
    return capabilities & V4L2_CAP_SLICED_VBI_CAPTURE;
}

bool capability::supports_sliced_vbi_output()
{
    return capabilities & V4L2_CAP_SLICED_VBI_OUTPUT;
}

bool capability::supports_rds_capture()
{
    return capabilities & V4L2_CAP_RDS_CAPTURE;
}

bool capability::supports_video_output_overlay()
{
    return capabilities & V4L2_CAP_VIDEO_OUTPUT_OVERLAY;
}

bool capability::supports_hardware_frequency_seeking()
{
    return capabilities & V4L2_CAP_HW_FREQ_SEEK;
}

bool capability::supports_rds_output()
{
    return capabilities & V4L2_CAP_RDS_OUTPUT;
}

bool capability::supports_tuner()
{
    return capabilities & V4L2_CAP_TUNER;
}

bool capability::supports_audio()
{
    return capabilities & V4L2_CAP_AUDIO;
}

bool capability::supports_radio()
{
    return capabilities & V4L2_CAP_RADIO;
}

bool capability::supports_modulator()
{
    return capabilities & V4L2_CAP_MODULATOR;
}

bool capability::supports_sdr_capture()
{
    return capabilities & V4L2_CAP_SDR_CAPTURE;
}

bool capability::supports_ext_pix_format()
{
    return capabilities & V4L2_CAP_EXT_PIX_FORMAT;
}

bool capability::supports_sdr_output()
{
    return capabilities & V4L2_CAP_SDR_OUTPUT;
}

bool capability::supports_meta_capture()
{
    return capabilities & V4L2_CAP_META_CAPTURE;
}

bool capability::supports_readwrite()
{
    return capabilities & V4L2_CAP_READWRITE;
}

bool capability::supports_streaming()
{
    return capabilities & V4L2_CAP_STREAMING;
}

bool capability::supports_meta_output()
{
    return capabilities & V4L2_CAP_META_OUTPUT;
}

bool capability::supports_touch()
{
    return capabilities & V4L2_CAP_TOUCH;
}

bool capability::supports_io_mc()
{
    return capabilities & V4L2_CAP_IO_MC;
}

bool capability::is_media_controller_centric()
{
    return device_caps & V4L2_CAP_IO_MC;
}

}  // namespace device
}  // namespace usb_cam