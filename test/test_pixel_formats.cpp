#include <gtest/gtest.h>

#include <linux/videodev2.h>

#include "usb_cam/formats/pixel_format_base.hpp"

TEST(test_pixel_formats, pixel_format_base_class) {
    auto test_pix_fmt = usb_cam::formats::default_pixel_format();

    EXPECT_EQ(test_pix_fmt.name(), "yuyv");
    EXPECT_EQ(test_pix_fmt.ros(), "yuv422_yuy2");
    EXPECT_EQ(test_pix_fmt.v4l2(), V4L2_PIX_FMT_YUYV);
    EXPECT_EQ(test_pix_fmt.channels(), 2);
    EXPECT_EQ(test_pix_fmt.bit_depth(), 8);

    EXPECT_EQ(test_pix_fmt.is_bayer(), false);
    EXPECT_EQ(test_pix_fmt.is_color(), true);
    EXPECT_EQ(test_pix_fmt.is_mono(), false);
}
