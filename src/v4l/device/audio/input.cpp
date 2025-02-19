#include <iostream>
#include "linux/videodev2.h"

#include "v4l/device/audio/input.hpp"


namespace v4l
{
namespace device
{
namespace audio
{

void Input::get_audio_input()
{
  memset(&m_input, 0, sizeof(m_input));

  if (query(VIDIOC_G_AUDIO, &m_input) == -1) {
    m_is_valid = false;
    return;
  }
  m_is_valid = true;
}

bool Input::is_audio_input_valid()
{
  return m_is_valid;
}

void Input::print_audio_input()
{
  std::cout << DIVIDER << std::endl;
  if (!is_audio_input_valid()) {
    std::cout << "No audio input available" << std::endl;
    return;
  }
  std::cout << "Current audio Input:" << std::endl;
  std::cout << "  Name: " << m_input.name << std::endl;
  std::cout << "  Index: " << m_input.index << std::endl;
  std::cout << "  Capabilities: " << std::endl;
}

bool Input::supports_stereo()
{
  return m_input.capability == V4L2_AUDCAP_STEREO;
}

bool Input::supports_avl()
{
  return m_input.capability == V4L2_AUDCAP_AVL;
}

bool Input::is_avl_on()
{
  return m_input.mode == V4L2_AUDMODE_AVL;
}

}  // namespace audio
}  // namespace device
}  // namespace v4l
