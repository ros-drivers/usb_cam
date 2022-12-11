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

#define CLEAR(x) memset(&(x), 0, sizeof(x))

extern "C" {
#include <linux/videodev2.h>  // Defines V4L2 format constants
#include <malloc.h>  // for memalign
#include <sys/mman.h>  // for mmap
#include <sys/stat.h>  // for stat
#define __STDC_CONSTANT_MACROS  // Required for libavutil
#include <libavutil/imgutils.h>
#include <fcntl.h>  // for O_* constants
#include <unistd.h>  // for getpagesize()
}

#include <chrono>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "opencv2/imgproc.hpp"

#include "usb_cam/usb_cam.hpp"
#include "usb_cam/conversions.hpp"
#include "usb_cam/utils.hpp"


namespace usb_cam
{

using utils::io_method_t;
using utils::pixel_format_t;
using utils::color_format_t;


UsbCam::UsbCam()
: io_(io_method_t::IO_METHOD_MMAP), fd_(-1), buffers_(NULL), n_buffers_(0),
  avframe_camera_(NULL), avframe_rgb_(NULL), avcodec_(NULL), avoptions_(NULL),
  avcodec_context_(NULL), avframe_camera_size_(0), avframe_rgb_size_(0),
  video_sws_(NULL), image_(NULL), is_capturing_(false),
  epoch_time_shift_(usb_cam::utils::get_epoch_time_shift()), supported_formats_()
{}

UsbCam::~UsbCam()
{
  shutdown();
}

int UsbCam::init_decoder(
  int image_width, int image_height, color_format_t color_format,
  AVCodecID codec_id, const char * codec_name)
{
  avcodec_ = avcodec_find_decoder(codec_id);
  if (!avcodec_) {
    std::cout << "Could not find " << codec_name << " decoder";
    return 0;
  }

  avcodec_context_ = avcodec_alloc_context3(avcodec_);

  // Suppress warnings of the following form:
  //
  // [swscaler @ 0x############] deprecated pixel format used, make sure you did set range correctly
  //
  // Or set this to AV_LOG_FATAL to additionally suppress occasional frame errors, e.g.:
  //
  // [mjpeg @ 0x############] overread 4
  // [mjpeg @ 0x############] No JPEG data found in image
  // [ERROR] [##########.##########]: Error while decoding frame.
  av_log_set_level(AV_LOG_ERROR);

#if LIBAVCODEC_VERSION_MAJOR < 55
  avframe_camera_ = avcodec_alloc_frame();
  avframe_rgb_ = avcodec_alloc_frame();
#else
  avframe_camera_ = av_frame_alloc();
  avframe_rgb_ = av_frame_alloc();
#endif

#if LIBAVCODEC_VERSION_MAJOR < 55
  avpicture_alloc(
    reinterpret_cast<AVPicture *>(avframe_rgb_), AV_PIX_FMT_RGB24, image_width, image_height);
#else
  av_image_alloc(
    reinterpret_cast<uint8_t **>(avframe_rgb_), 0, image_width, image_height, AV_PIX_FMT_RGB24, 1);
#endif
  avcodec_context_->codec_id = AV_CODEC_ID_MJPEG;
  avcodec_context_->width = image_width;
  avcodec_context_->height = image_height;

#if LIBAVCODEC_VERSION_MAJOR > 52
  // TODO(lucasw) it gets set correctly here, but then changed later to deprecated J422P format
  if (color_format == color_format_t::COLOR_FORMAT_YUV420P) {
    avcodec_context_->pix_fmt = AV_PIX_FMT_YUV420P;
    std::cout << "using YUV420P " << AV_PIX_FMT_YUV420P << " ";
    std::cout << avcodec_context_->pix_fmt << std::endl;
  } else {
    avcodec_context_->pix_fmt = AV_PIX_FMT_YUV422P;
    std::cout << "using YUV422P " << AV_PIX_FMT_YUV422P << " ";
    std::cout << avcodec_context_->pix_fmt << std::endl;
  }

  avcodec_context_->codec_type = AVMEDIA_TYPE_VIDEO;
#endif

#if LIBAVCODEC_VERSION_MAJOR < 55
  avframe_camera_size_ = avpicture_get_size(AV_PIX_FMT_YUV422P, image_width, image_height);
  avframe_rgb_size_ = avpicture_get_size(AV_PIX_FMT_RGB24, image_width, image_height);
#else
  avframe_camera_size_ =
    av_image_get_buffer_size(AV_PIX_FMT_YUV422P, image_width, image_height, 1);
  avframe_rgb_size_ =
    av_image_get_buffer_size(AV_PIX_FMT_RGB24, image_width, image_height, 1);
#endif

  /* open it */
  if (avcodec_open2(avcodec_context_, avcodec_, &avoptions_) < 0) {
    std::cout << "Could not open " << codec_name << " decoder" << std::endl;
    return 0;
  }
  std::cout << "pixel format " << AV_PIX_FMT_YUV422P << " ";
  std::cout << avcodec_context_->pix_fmt << std::endl;
  return 1;
}

int UsbCam::init_mjpeg_decoder(int image_width, int image_height, color_format_t color_format)
{
  return init_decoder(image_width, image_height, color_format, AV_CODEC_ID_MJPEG, "MJPEG");
}

int UsbCam::init_h264_decoder(int image_width, int image_height, color_format_t color_format)
{
  return init_decoder(image_width, image_height, color_format, AV_CODEC_ID_H264, "H264");
}

bool UsbCam::process_image(const void * src, int len, camera_image_t * dest)
{
  bool result = false;
  if (pixelformat_ == V4L2_PIX_FMT_YUYV) {
    if (monochrome_) {
      // actually format V4L2_PIX_FMT_Y16, but usb_cam::utils::xioctl gets unhappy
      // if you don't use the advertised type (yuyv)
      result = conversions::MONO102MONO8(
        const_cast<char *>(
          reinterpret_cast<const char *>(src)), dest->image, dest->width * dest->height);
    } else {
      result = conversions::YUYV2RGB(
        const_cast<char *>(
          reinterpret_cast<const char *>(src)), dest->image, dest->width * dest->height);
    }
  } else if (pixelformat_ == V4L2_PIX_FMT_UYVY) {
    result = conversions::UYVY2RGB(
      const_cast<char *>(
        reinterpret_cast<const char *>(src)), dest->image, dest->width * dest->height);
  } else if (pixelformat_ == V4L2_PIX_FMT_MJPEG || pixelformat_ == V4L2_PIX_FMT_H264) {
    result = conversions::MJPEG2RGB(
      this,
      const_cast<char *>(
        reinterpret_cast<const char *>(src)), len, dest->image, dest->width * dest->height);
  } else if (pixelformat_ == V4L2_PIX_FMT_RGB24 || pixelformat_ == V4L2_PIX_FMT_GREY) {
    result = conversions::COPY2RGB(
      const_cast<char *>(
        reinterpret_cast<const char *>(src)), dest->image, dest->width * dest->height);
  } else if (pixelformat_ == V4L2_PIX_FMT_YUV420) {
    result = conversions::YUV4202RGB(
      const_cast<char *>(
        reinterpret_cast<const char *>(src)), dest->image, dest->width, dest->height);
  }

  return result;
}

bool UsbCam::read_frame()
{
  struct v4l2_buffer buf;
  unsigned int i;
  int len;
  struct timespec stamp;
  int64_t buffer_time_s;

  switch (io_) {
    case io_method_t::IO_METHOD_READ:
      len = read(fd_, buffers_[0].start, buffers_[0].length);
      if (len == -1) {
        switch (errno) {
          case EAGAIN:
            return false;

          case EIO:
          /* Could ignore EIO, see spec. */

          /* fall through */

          default:
            std::cerr << "Unable to read frame " << errno << std::endl;
            return false;  // ("read");
        }
      }

      if (!process_image(buffers_[0].start, len, image_)) {
        return false;
      }
      // TODO(lucasw) how to get timestamp with this method?

      break;

    case io_method_t::IO_METHOD_MMAP:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if (-1 == usb_cam::utils::xioctl(fd_, static_cast<int>(VIDIOC_DQBUF), &buf)) {
        switch (errno) {
          case EAGAIN:
            return false;

          case EIO:
          /* Could ignore EIO, see spec. */

          /* fall through */

          default:
            std::cerr << "Unable to retrieve frame with mmap " << errno << std::endl;
            return false;  // ("VIDIOC_DQBUF");
        }
      }

      buffer_time_s =
        buf.timestamp.tv_sec + static_cast<int64_t>(round(buf.timestamp.tv_usec / 1000000.0));

      stamp.tv_sec = static_cast<time_t>(round(buffer_time_s)) + epoch_time_shift_;
      stamp.tv_nsec = static_cast<int64_t>(buf.timestamp.tv_usec * 1000.0);

      assert(buf.index < n_buffers_);
      len = buf.bytesused;
      if (!process_image(buffers_[buf.index].start, len, image_)) {
        return false;
      }

      if (-1 == usb_cam::utils::xioctl(fd_, static_cast<int>(VIDIOC_QBUF), &buf)) {
        std::cerr << "Unable to exchange buffer with the driver " << errno << std::endl;
        return false;  // ("VIDIOC_QBUF");
      }

      image_->stamp = stamp;

      break;

    case io_method_t::IO_METHOD_USERPTR:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == usb_cam::utils::xioctl(fd_, static_cast<int>(VIDIOC_DQBUF), &buf)) {
        switch (errno) {
          case EAGAIN:
            return false;

          case EIO:
          /* Could ignore EIO, see spec. */

          /* fall through */

          default:
            std::cerr << "Unable to exchange buffer with driver " << errno << std::endl;
            return false;  // ("VIDIOC_DQBUF");
        }
      }

      buffer_time_s =
        buf.timestamp.tv_sec + static_cast<int64_t>(round(buf.timestamp.tv_usec / 1000000.0));

      stamp.tv_sec = static_cast<time_t>(round(buffer_time_s)) + epoch_time_shift_;
      stamp.tv_nsec = static_cast<int64_t>(buf.timestamp.tv_usec / 1000.0);

      for (i = 0; i < n_buffers_; ++i) {
        if (buf.m.userptr == reinterpret_cast<uint64_t>(buffers_[i].start) && \
          buf.length == buffers_[i].length)
        {
          break;
        }
      }

      assert(i < n_buffers_);
      len = buf.bytesused;
      if (!process_image(reinterpret_cast<const void *>(buf.m.userptr), len, image_)) {
        return false;
      }

      if (-1 == usb_cam::utils::xioctl(fd_, static_cast<int>(VIDIOC_QBUF), &buf)) {
        std::cerr << "Unable to exchange buffer with driver " << errno << std::endl;
        return false;  // ("VIDIOC_QBUF");
      }

      image_->stamp = stamp;
      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      // TODO(flynneva): log something to indicate IO method unknown
      break;
  }

  return true;
}

bool UsbCam::stop_capturing(void)
{
  if (!is_capturing_) {return false;}

  is_capturing_ = false;
  enum v4l2_buf_type type;

  switch (io_) {
    case io_method_t::IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case io_method_t::IO_METHOD_MMAP:
    case io_method_t::IO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == usb_cam::utils::xioctl(fd_, VIDIOC_STREAMOFF, &type)) {
        std::cerr << "Unable to stop capturing stream " << errno << std::endl;
        return false;  // ("VIDIOC_STREAMOFF");
      }

      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      // TODO(flynneva): log something indicating IO method unknown
      break;
  }
  return true;
}

bool UsbCam::start_capturing(void)
{
  if (is_capturing_) {return false;}

  unsigned int i;
  enum v4l2_buf_type type;

  switch (io_) {
    case io_method_t::IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case io_method_t::IO_METHOD_MMAP:
      for (i = 0; i < n_buffers_; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == usb_cam::utils::xioctl(fd_, static_cast<int>(VIDIOC_QBUF), &buf)) {
          std::cerr << "Unable to configure strem " << errno << std::endl;
          return false;  // ("VIDIOC_QBUF");
        }
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == usb_cam::utils::xioctl(fd_, VIDIOC_STREAMON, &type)) {
        std::cerr << "Unable to start stream " << errno << std::endl;
        return false;  // ("VIDIOC_STREAMON");
      }
      break;

    case io_method_t::IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers_; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;
        buf.m.userptr = reinterpret_cast<uint64_t>(buffers_[i].start);
        buf.length = buffers_[i].length;

        if (-1 == usb_cam::utils::xioctl(fd_, static_cast<int>(VIDIOC_QBUF), &buf)) {
          std::cerr << "Unable to configure stream " << errno << std::endl;
          return false;  // ("VIDIOC_QBUF");
        }
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == usb_cam::utils::xioctl(fd_, VIDIOC_STREAMON, &type)) {
        std::cerr << "Unable to start stream " << errno << std::endl;
        return false;  // ("VIDIOC_STREAMON");
      }

      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      // TODO(flynneva): log something indicating IO method unknown
      break;
  }
  is_capturing_ = true;
  return true;
}

bool UsbCam::uninit_device(void)
{
  unsigned int i;

  switch (io_) {
    case io_method_t::IO_METHOD_READ:
      free(buffers_[0].start);
      break;

    case io_method_t::IO_METHOD_MMAP:
      for (i = 0; i < n_buffers_; ++i) {
        if (-1 == munmap(buffers_[i].start, buffers_[i].length)) {
          std::cerr << "Unable to deallocate memory " << errno << std::endl;
          return false;  // ("munmap");
        }
      }
      break;

    case io_method_t::IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers_; ++i) {
        free(buffers_[i].start);
      }
      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      // TODO(flynneva): log something
      break;
  }

  free(buffers_);
  return true;
}

bool UsbCam::init_read(unsigned int buffer_size)
{
  buffers_ = reinterpret_cast<usb_cam::utils::buffer *>(calloc(1, sizeof(*buffers_)));

  if (!buffers_) {
    std::cerr << "Out of memory" << std::endl;
    return false;
  }

  buffers_[0].length = buffer_size;
  buffers_[0].start = malloc(buffer_size);

  if (!buffers_[0].start) {
    std::cerr << "Out of memory" << std::endl;
    return false;
  }
  return true;
}

bool UsbCam::init_mmap(void)
{
  struct v4l2_requestbuffers req;

  CLEAR(req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == usb_cam::utils::xioctl(fd_, static_cast<int>(VIDIOC_REQBUFS), &req)) {
    if (EINVAL == errno) {
      std::cerr << camera_dev_ << " does not support memory mapping" << std::endl;
      return false;
    } else {
      std::cerr << "Unable to initialize memory mapping " << errno << std::endl;
      return false;  // ("VIDIOC_REQBUFS");
    }
  }

  if (req.count < 2) {
    std::cerr << "Insufficient buffer memory on " << camera_dev_ << std::endl;
    return false;
  }

  buffers_ = reinterpret_cast<usb_cam::utils::buffer *>(calloc(req.count, sizeof(*buffers_)));

  if (!buffers_) {
    std::cerr << "Out of memory" << std::endl;
    return false;
  }

  for (n_buffers_ = 0; n_buffers_ < req.count; ++n_buffers_) {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers_;

    if (-1 == usb_cam::utils::xioctl(fd_, static_cast<int>(VIDIOC_QUERYBUF), &buf)) {
      std::cerr << "Unable to query status of buffer " << errno << std::endl;
      return false;  // ("VIDIOC_QUERYBUF");
    }

    buffers_[n_buffers_].length = buf.length;
    buffers_[n_buffers_].start =
      mmap(
      NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */,
      MAP_SHARED /* recommended */, fd_, buf.m.offset);

    if (MAP_FAILED == buffers_[n_buffers_].start) {
      std::cerr << "Unable to allocate memory " << errno << std::endl;
      return false;  // ("mmap");
    }
  }
  return true;
}

bool UsbCam::init_userp(unsigned int buffer_size)
{
  struct v4l2_requestbuffers req;
  unsigned int page_size;

  page_size = getpagesize();
  buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

  CLEAR(req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;

  if (-1 == usb_cam::utils::xioctl(fd_, static_cast<int>(VIDIOC_REQBUFS), &req)) {
    if (EINVAL == errno) {
      std::cerr << camera_dev_ << " does not support user pointer i/o" << std::endl;
      return false;  // (EXIT_FAILURE);
    } else {
      std::cerr << "Unable to initialize memory mapping " << errno << std::endl;
      return false;  // ("VIDIOC_REQBUFS");
    }
  }

  buffers_ = reinterpret_cast<usb_cam::utils::buffer *>(calloc(4, sizeof(*buffers_)));

  if (!buffers_) {
    std::cerr << "Out of memory" << std::endl;
    return false;  // (EXIT_FAILURE);
  }

  for (n_buffers_ = 0; n_buffers_ < 4; ++n_buffers_) {
    buffers_[n_buffers_].length = buffer_size;
    buffers_[n_buffers_].start = memalign(/* boundary */ page_size, buffer_size);

    if (!buffers_[n_buffers_].start) {
      std::cerr << "Out of memory" << std::endl;
      return false;
    }
  }
  return true;
}

bool UsbCam::init_device(uint32_t image_width, uint32_t image_height, int framerate)
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;

  if (-1 == usb_cam::utils::xioctl(fd_, static_cast<int>(VIDIOC_QUERYCAP), &cap)) {
    if (EINVAL == errno) {
      std::cerr << camera_dev_ << " is no V4L2 device" << std::endl;
      return false;  // (EXIT_FAILURE);
    } else {
      std::cerr << "Unable to query device capabilities " << errno << std::endl;
      return false;  // ("VIDIOC_QUERYCAP");
    }
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    std::cerr << camera_dev_ << " is no video capture device" << std::endl;
    return false;  // (EXIT_FAILURE);
  }

  switch (io_) {
    case io_method_t::IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
        std::cerr << camera_dev_ << " does not support read i/o" << std::endl;
        return false;  // (EXIT_FAILURE);
      }

      break;

    case io_method_t::IO_METHOD_MMAP:
    case io_method_t::IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        std::cout << camera_dev_ << " does not support streaming i/o" << std::endl;
        return false;  // (EXIT_FAILURE);
      }

      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      // TODO(flynneva): log something
      break;
  }

  /* Select video input, video standard and tune here. */

  CLEAR(cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0 == usb_cam::utils::xioctl(fd_, static_cast<int>(VIDIOC_CROPCAP), &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1 == usb_cam::utils::xioctl(fd_, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
        case EINVAL:
          /* Cropping not supported. */
          break;
        default:
          /* Errors ignored. */
          break;
      }
    }
  } else {
    /* Errors ignored. */
  }

  CLEAR(fmt);

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = image_width;
  fmt.fmt.pix.height = image_height;
  fmt.fmt.pix.pixelformat = pixelformat_;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  if (-1 == usb_cam::utils::xioctl(fd_, static_cast<int>(VIDIOC_S_FMT), &fmt)) {
    std::cerr << "Unable to set video format " << errno << std::endl;
    return false;  // ("VIDIOC_S_FMT");
  }

  /* Note VIDIOC_S_FMT may change width and height. */

  /* Buggy driver paranoia. */
  min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min) {
    fmt.fmt.pix.bytesperline = min;
  }

  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min) {
    fmt.fmt.pix.sizeimage = min;
  }

  image_width = fmt.fmt.pix.width;
  image_height = fmt.fmt.pix.height;

  struct v4l2_streamparm stream_params;
  memset(&stream_params, 0, sizeof(stream_params));
  stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (usb_cam::utils::xioctl(fd_, static_cast<int>(VIDIOC_G_PARM), &stream_params) < 0) {
    std::cerr << "can't set stream params " << errno << std::endl;
    return false;  // ("Couldn't query v4l fps!");
  }

  if (!stream_params.parm.capture.capability && V4L2_CAP_TIMEPERFRAME) {
    std::cerr << "V4L2_CAP_TIMEPERFRAME not supported" << std::endl;
  }

  // TODO(lucasw) need to get list of valid numerator/denominator pairs
  // and match closest to what user put in.
  stream_params.parm.capture.timeperframe.numerator = 1;
  stream_params.parm.capture.timeperframe.denominator = framerate;
  if (usb_cam::utils::xioctl(fd_, static_cast<int>(VIDIOC_S_PARM), &stream_params) < 0) {
    std::cerr << "Couldn't set camera framerate" << std::endl;
  }

  switch (io_) {
    case io_method_t::IO_METHOD_READ:
      init_read(fmt.fmt.pix.sizeimage);
      break;

    case io_method_t::IO_METHOD_MMAP:
      init_mmap();
      break;

    case io_method_t::IO_METHOD_USERPTR:
      init_userp(fmt.fmt.pix.sizeimage);
      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      // TODO(flynneva): log something
      break;
  }
  return true;
}

bool UsbCam::close_device(void)
{
  if (-1 == close(fd_)) {
    std::cerr << "Unable to close file descriptor " << errno << std::endl;
    return false;  // ("close");
  }

  fd_ = -1;
  return true;
}

bool UsbCam::open_device(void)
{
  struct stat st;

  if (-1 == stat(camera_dev_.c_str(), &st)) {
    std::cerr << "Cannot identify '" << camera_dev_ << "': " << errno << ", ";
    std::cerr << strerror(errno) << std::endl;
    return false;  // (EXIT_FAILURE);
  }

  if (!S_ISCHR(st.st_mode)) {
    std::cerr << camera_dev_ << " is no device" << std::endl;
    return false;  // (EXIT_FAILURE);
  }

  fd_ = open(camera_dev_.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);

  if (-1 == fd_) {
    std::cerr << "Cannot open '" << camera_dev_ << "': " << errno << ", ";
    std::cerr << strerror(errno) << std::endl;
    return false;  // (EXIT_FAILURE);
  }
  return true;
}

bool UsbCam::start(
  const std::string & dev,
  io_method_t io_method, pixel_format_t pixel_format, color_format_t color_format,
  uint32_t image_width, uint32_t image_height, int framerate)
{
  camera_dev_ = dev;

  io_ = io_method;
  monochrome_ = false;
  if (pixel_format == pixel_format_t::PIXEL_FORMAT_YUYV) {
    pixelformat_ = V4L2_PIX_FMT_YUYV;
  } else if (pixel_format == pixel_format_t::PIXEL_FORMAT_UYVY) {
    pixelformat_ = V4L2_PIX_FMT_UYVY;
  } else if (pixel_format == pixel_format_t::PIXEL_FORMAT_MJPEG) {
    pixelformat_ = V4L2_PIX_FMT_MJPEG;
    init_mjpeg_decoder(image_width, image_height, color_format);
  } else if (pixel_format == pixel_format_t::PIXEL_FORMAT_H264) {
    pixelformat_ = V4L2_PIX_FMT_H264;
    init_h264_decoder(image_width, image_height, color_format);
  } else if (pixel_format == pixel_format_t::PIXEL_FORMAT_YUVMONO10) {
    // actually format V4L2_PIX_FMT_Y16 (10-bit mono expresed as 16-bit pixels)
    // but we need to use the advertised type (yuyv)
    pixelformat_ = V4L2_PIX_FMT_YUYV;
    monochrome_ = true;
  } else if (pixel_format == pixel_format_t::PIXEL_FORMAT_RGB24) {
    pixelformat_ = V4L2_PIX_FMT_RGB24;
  } else if (pixel_format == pixel_format_t::PIXEL_FORMAT_GREY) {
    pixelformat_ = V4L2_PIX_FMT_GREY;
    monochrome_ = true;
  } else if (pixel_format == pixel_format_t::PIXEL_FORMAT_YU12) {
    pixelformat_ = V4L2_PIX_FMT_YUV420;
  } else {
    std::cerr << "Unknown pixel format" << std::endl;
    return false;  // (EXIT_FAILURE);
  }

  // TODO(lucasw) throw exceptions instead of return value checking
  if (!open_device()) {
    return false;
  }
  if (!init_device(image_width, image_height, framerate)) {
    return false;
  }
  if (!start_capturing()) {
    return false;
  }

  image_ = reinterpret_cast<camera_image_t *>(calloc(1, sizeof(camera_image_t)));

  image_->width = image_width;
  image_->height = image_height;
  image_->bytes_per_pixel = 3;  // corrected 11/10/15 (BYTES not BITS per pixel)

  image_->image_size = image_->width * image_->height * image_->bytes_per_pixel;
  image_->is_new = 0;
  image_->image = reinterpret_cast<char *>(calloc(image_->image_size, sizeof(char *)));
  memset(image_->image, 0, image_->image_size * sizeof(char *));
  return true;
}

bool UsbCam::shutdown(void)
{
  stop_capturing();
  uninit_device();
  close_device();

  if (avcodec_context_) {
    avcodec_close(avcodec_context_);
    av_free(avcodec_context_);
    avcodec_context_ = NULL;
  }
  if (avframe_camera_) {
    av_free(avframe_camera_);
  }
  avframe_camera_ = NULL;
  if (avframe_rgb_) {
    av_free(avframe_rgb_);
  }
  avframe_rgb_ = NULL;
  if (image_) {
    free(image_);
  }
  image_ = NULL;
  return true;
}

camera_image_t * UsbCam::get_image()
{
  if ((image_->width == 0) || (image_->height == 0)) {
    return nullptr;
  }
  // grab the image
  if (!grab_image()) {
    return nullptr;
  }

  if (monochrome_) {
    image_->encoding = "mono8";
    image_->step = image_->width;
  } else {
    // TODO(lucasw) aren't there other encoding types?
    image_->encoding = "rgb8";
    image_->step = image_->width * 3;
  }

  return image_;
}

std::vector<capture_format_t> UsbCam::get_supported_formats()
{
  supported_formats_.clear();
  struct v4l2_fmtdesc current_format;
  current_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  current_format.index = 0;
  for (current_format.index = 0;
    usb_cam::utils::xioctl(
      fd_, static_cast<int>(VIDIOC_ENUM_FMT), &current_format) == 0;
    ++current_format.index)
  {
    struct v4l2_frmsizeenum current_size;
    current_size.index = 0;
    current_size.pixel_format = current_format.pixelformat;

    for (current_size.index = 0;
      usb_cam::utils::xioctl(
        fd_, static_cast<int>(VIDIOC_ENUM_FRAMESIZES), &current_size) == 0;
      ++current_size.index)
    {
      struct v4l2_frmivalenum current_interval;
      current_interval.index = 0;
      current_interval.pixel_format = current_size.pixel_format;
      current_interval.width = current_size.discrete.width;
      current_interval.height = current_size.discrete.height;
      for (current_interval.index = 0;
        usb_cam::utils::xioctl(
          fd_, static_cast<int>(VIDIOC_ENUM_FRAMEINTERVALS), &current_interval) == 0;
        ++current_interval.index)
      {
        if (current_interval.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
          capture_format_t capture_format;
          capture_format.format = current_format;
          capture_format.size = current_size;
          capture_format.interval = current_interval;
          supported_formats_.push_back(capture_format);
        }
      }  // interval loop
    }  // size loop
  }  // fmt loop

  return supported_formats_;
}

bool UsbCam::grab_image()
{
  fd_set fds;
  struct timeval tv;
  int r;

  FD_ZERO(&fds);
  FD_SET(fd_, &fds);

  /* Timeout. */
  tv.tv_sec = 5;
  tv.tv_usec = 0;

  r = select(fd_ + 1, &fds, NULL, NULL, &tv);
  // if the v4l2_buffer timestamp isn't available use this time, though
  // it may be 10s of milliseconds after the frame acquisition.
  // image_->stamp = clock_->now();
  timespec_get(&image_->stamp, TIME_UTC);

  if (-1 == r) {
    if (EINTR == errno) {
      return false;
    }

    std::cerr << "Something went wrong, exiting..." << errno << std::endl;
    return false;  // ("select");
  }

  if (0 == r) {
    std::cerr << "Select timeout, exiting..." << std::endl;
    return false;
  }

  if (!read_frame()) {
    return false;
  }
  image_->is_new = 1;
  return true;
}

// enables/disables auto focus
bool UsbCam::set_auto_focus(int value)
{
  struct v4l2_queryctrl queryctrl;
  struct v4l2_ext_control control;

  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = V4L2_CID_FOCUS_AUTO;

  if (-1 == usb_cam::utils::xioctl(fd_, static_cast<int>(VIDIOC_QUERYCTRL), &queryctrl)) {
    if (errno != EINVAL) {
      std::cerr << "VIDIOC_QUERYCTRL" << std::endl;
      return false;
    } else {
      std::cerr << "V4L2_CID_FOCUS_AUTO is not supported" << std::endl;
      return false;
    }
  } else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
    std::cerr << "V4L2_CID_FOCUS_AUTO is not supported" << std::endl;
    return false;
  } else {
    memset(&control, 0, sizeof(control));
    control.id = V4L2_CID_FOCUS_AUTO;
    control.value = value;

    if (-1 == usb_cam::utils::xioctl(fd_, static_cast<int>(VIDIOC_S_CTRL), &control)) {
      std::cerr << "VIDIOC_S_CTRL" << std::endl;
      return false;
    }
  }
  return true;
}

/**
* Set video device parameter via call to v4l-utils.
*
* @param param The name of the parameter to set
* @param param The value to assign
*/
bool UsbCam::set_v4l_parameter(const std::string & param, int value)
{
  char buf[33];
  snprintf(buf, sizeof(buf), "%i", value);
  return set_v4l_parameter(param, buf);
}

/**
* Set video device parameter via call to v4l-utils.
*
* @param param The name of the parameter to set
* @param param The value to assign
*/
bool UsbCam::set_v4l_parameter(const std::string & param, const std::string & value)
{
  int retcode = 0;
  // build the command
  std::stringstream ss;
  ss << "v4l2-ctl --device=" << camera_dev_ << " -c " << param << "=" << value << " 2>&1";
  std::string cmd = ss.str();

  // capture the output
  std::string output;
  const int kBufferSize = 256;
  char buffer[kBufferSize];
  FILE * stream = popen(cmd.c_str(), "r");
  if (stream) {
    while (!feof(stream)) {
      if (fgets(buffer, kBufferSize, stream) != NULL) {
        output.append(buffer);
      }
    }
    pclose(stream);
    // any output should be an error
    if (output.length() > 0) {
      std::cout << output.c_str() << std::endl;
      retcode = 1;
    }
  } else {
    std::cerr << "usb_cam_node could not run '" << cmd.c_str() << "'" << std::endl;
    retcode = 1;
  }
  return retcode;
}

}  // namespace usb_cam
