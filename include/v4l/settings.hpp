#ifndef V4L__SETTINGS_HPP_
#define V4L__SETTINGS_HPP_
/// @brief Utilities for working with V4L2 user controls.
///
/// For example, listing all available controls for the current device
/// or setting a value of a control

extern "C" {
#include <libavcodec/avcodec.h>
#include <linux/videodev2.h>
}

#include <sys/ioctl.h>

struct v4l2_queryctrl queryctrl;
struct v4l2_querymenu querymenu;

static void enumerate_menu(int fd)
{
  printf("  Menu items:\\n");

  memset(&querymenu, 0, sizeof(querymenu));
  querymenu.id = queryctrl.id;

  for (querymenu.index = queryctrl.minimum;
    querymenu.index <= queryctrl.maximum;
    querymenu.index++)
  {
    if (0 == ioctl(fd, VIDIOC_QUERYMENU, &querymenu)) {
      printf("  %s\\n", querymenu.name);
    }
  }
}

#endif  // V4L__SETTINGS_HPP_
