#include <iostream>
#include "linux/videodev2.h"

#include "v4l/device/audio/output.hpp"


namespace v4l
{
namespace device
{
namespace audio
{

void Output::get_audio_output()
{
  memset(&m_output, 0, sizeof(m_output));

  if (query(VIDIOC_G_AUDOUT, &m_output) == -1) {
    m_is_valid = false;
    return;
  }
  m_is_valid = true;
}

bool Output::is_audio_output_valid()
{
  return m_is_valid;
}

void Output::print_audio_output()
{
  std::cout << DIVIDER << std::endl;
  if (!is_audio_output_valid()) {
    std::cout << "No audio output available" << std::endl;
    return;
  }
  std::cout << "Current audio output:" << std::endl;
  std::cout << "  Name: " << m_output.name << std::endl;
  std::cout << "  Index: " << m_output.index << std::endl;
  std::cout << "  Capabilities: " << std::endl;
}

bool Output::supports_stereo()
{
  return m_output.capability == V4L2_AUDCAP_STEREO;
}

bool Output::supports_avl()
{
  return m_output.capability == V4L2_AUDCAP_AVL;
}

bool Output::is_avl_on()
{
  return m_output.mode == V4L2_AUDMODE_AVL;
}

}  // namespace audio
}  // namespace device
}  // namespace v4l
