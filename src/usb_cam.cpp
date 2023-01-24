/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <linux/videodev2.h>
#include <ros/ros.h>
#include <sstream>
#include "usb_cam/usb_cam.h"

using namespace usb_cam;

/* STATIC DATA INITIALIZERS */
/* V4L/HARDWARE */
io_method_t UsbCam::io_method = io_method_t::IO_METHOD_MMAP; // io_
pixel_format_t UsbCam::pixel_format = PIXEL_FORMAT_UNKNOWN;
color_format_t UsbCam::color_format = COLOR_FORMAT_UNKNOWN;
bool UsbCam::monochrome = false;
int UsbCam::file_dev = -1;
const time_t UsbCam::epoch_time_shift = util::get_epoch_time_shift();
bool UsbCam::create_suspended = false;

/* FFMPEG */
bool UsbCam::full_ffmpeg_log = false;
buffer* UsbCam::buffers = nullptr;
unsigned int UsbCam::buffers_count = 0; // n_buffers_
AVFrame* UsbCam::avframe_camera = nullptr;
AVFrame* UsbCam::avframe_rgb = nullptr;
AVPacket* UsbCam::avpkt = nullptr;
AVCodec* UsbCam::avcodec = nullptr;
AVCodecID UsbCam::codec_id = AV_CODEC_ID_NONE;
AVDictionary* UsbCam::avoptions = nullptr;
AVCodecContext* UsbCam::avcodec_context = nullptr;
int UsbCam::avframe_camera_size = 0;
int UsbCam::avframe_rgb_size = 0;
struct SwsContext* UsbCam::video_sws = nullptr;
camera_image_t* UsbCam::image = nullptr;
bool UsbCam::capturing = false;
std::vector<capture_format_t> UsbCam::supported_formats = std::vector<capture_format_t>();

/* ROS */
ros::Timer* UsbCam::frame_timer = nullptr;
sensor_msgs::Image* UsbCam::img_msg = nullptr;
image_transport::CameraPublisher* UsbCam::image_pub = nullptr;
camera_info_manager::CameraInfoManager* UsbCam::camera_info = nullptr;
ros::ServiceServer* UsbCam::service_start = nullptr;
ros::ServiceServer* UsbCam::service_stop = nullptr;
ros::ServiceServer* UsbCam::service_supported_formats = nullptr;
image_transport::ImageTransport* UsbCam::image_transport = nullptr;

/* V4L camera parameters */
bool UsbCam::streaming_status = false;
std::string UsbCam::video_device_name = "/dev/video0";
std::string UsbCam::io_method_name = "mmap";
std::string UsbCam::pixel_format_name = "uyvy";
unsigned int UsbCam::v4l_pixel_format = V4L2_PIX_FMT_UYVY;
std::string UsbCam::color_format_name = "yuv422p";
std::string UsbCam::camera_name = "head_camera";
std::string UsbCam::camera_frame_id = "head_camera";
std::string UsbCam::camera_transport_suffix = "image_raw";
std::string UsbCam::camera_info_url = "";
int UsbCam::image_width = 320;
int UsbCam::image_height = 240;
int UsbCam::framerate = 10;
int UsbCam::exposure = 100;
int UsbCam::brightness = -1;
int UsbCam::contrast = -1;
int UsbCam::saturation = -1;
int UsbCam::sharpness = -1;
int UsbCam::focus = -1;
int UsbCam::white_balance = 4000;
int UsbCam::gain = -1;
bool UsbCam::autofocus = false;
bool UsbCam::autoexposure = true;
bool UsbCam::auto_white_balance = true;

/* ROS Service callback functions */
bool UsbCam::service_start_callback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    return start_capture();
}

bool UsbCam::service_stop_callback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    return suspend();
}

bool UsbCam::service_supported_formats_callback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{
    get_supported_formats();
    std::stringstream output_stream;
    std::cout << "SUPPORTED INPUT FORMATS FOR V4L DEVICE " << video_device_name << std::endl;
    for(auto fmt : supported_formats)
    {
        output_stream << " | " << fmt.format.description
                      << " [" << fmt.interval.width << " x "
                      << fmt.interval.height << "], "
                      << fmt.interval.discrete.denominator / fmt.interval.discrete.numerator
                      << " fps";
        std::cout << "\t" << fmt.format.description
                  << " [" << fmt.interval.width << " x "
                  << fmt.interval.height << "], "
                  << fmt.interval.discrete.denominator / fmt.interval.discrete.numerator
                  << " fps" << std::endl;
    }
    response.success = true;
    response.message = output_stream.str();
    return true;
}

bool UsbCam::start()
{
    // V4L initilaization data
    struct stat st;
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    unsigned int min;
    struct v4l2_streamparm stream_params;

    /* Creating filesystem handler for streaming device */
    ROS_INFO("Opening streaming device %s", video_device_name.c_str());
    if(stat(video_device_name.c_str(), &st) < 0)
    {
        ROS_ERROR("Cannot identify device by name '%s' (%i)", video_device_name.c_str(), errno);
        return false;
    }
    if(!S_ISCHR(st.st_mode))
    {
        ROS_ERROR("'%s' is not a proper V4L device (%i)", video_device_name.c_str(), errno);
        return false;
    }
    file_dev = open(video_device_name.c_str(),
                    O_RDWR|O_NONBLOCK,
                    0);
    if(file_dev < 0)
    {
        ROS_ERROR("Cannot create a file handler for V4L device '%s' (%i)", video_device_name.c_str(), errno);
        return false;
    }

    /* Initializing V4L capture pipeline */
    if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_QUERYCAP), &cap) < 0)
    {
        if(errno == EINVAL)
            ROS_ERROR("File handler created for V4L-incompatible device '%s' (%i)", video_device_name.c_str(), errno);
        else
            ROS_ERROR("Cannot query capabilities from V4L device '%s' (%i)", video_device_name.c_str(), errno);
        return false;
    }
    if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        ROS_ERROR("V4L device '%s' does not support capture mode (%i)", video_device_name.c_str(), errno);
        return false;
    }
    switch(io_method)
    {
    case IO_METHOD_READ:
        if(!(cap.capabilities & V4L2_CAP_READWRITE))
        {
            ROS_ERROR("Device '%s' does not support '%s' access method (read/write error)", video_device_name.c_str(), io_method_name.c_str());
            return false;
        }
        break;
    case io_method_t::IO_METHOD_MMAP:
    case io_method_t::IO_METHOD_USERPTR:
        if(!(cap.capabilities & V4L2_CAP_STREAMING))
        {
            ROS_ERROR("Device '%s' does not support '%s' access method (streaming error)", video_device_name.c_str(), io_method_name.c_str());
            return false;
        }
        break;
    default:
        ROS_ERROR("Cannot parse access mode for device '%s': '%s', system malfunction expected", video_device_name.c_str(), io_method_name.c_str());
    }
    /* V4L pipeline tuning */
    CLEAR(cropcap);
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_CROPCAP), &cropcap) == 0)
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */
        if(usb_cam::util::xioctl(file_dev, VIDIOC_S_CROP, &crop) < 0)
        {
            if(errno == EINVAL)
                ROS_WARN("Video4Linux: CROP  mode is not supported");
            else
                ROS_WARN("Video4Linux: IOCTL is not supported");
        }
    }
    else
        ROS_ERROR("Video4Linux: internal error occurred, hoping for device fallback");
    CLEAR(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = image_width;
    fmt.fmt.pix.height = image_height;
    fmt.fmt.pix.pixelformat = v4l_pixel_format;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_S_FMT), &fmt) < 0)
    {
        ROS_ERROR("Cannot set pixel format '%s' (%u)", pixel_format_name.c_str(), v4l_pixel_format);
        return false;
    }
    // Buggy driver prevention
    min = fmt.fmt.pix.width * 2;
    if(fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if(fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;
    image_width = fmt.fmt.pix.width;
    image_height = fmt.fmt.pix.height;
    CLEAR(stream_params);
    stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_G_PARM), &stream_params) < 0)
    {
        ROS_ERROR("Cannot set stream parameters (%i)", errno);
        return false;
    }
    if(!stream_params.parm.capture.capability && V4L2_CAP_TIMEPERFRAME)
        ROS_ERROR("Video4Linux: V4L2_CAP_TIMEPERFRAME not supported");
    // TODO(lucasw) need to get list of valid numerator/denominator pairs
    // and match closest to what user put in.
    stream_params.parm.capture.timeperframe.numerator = 1;
    stream_params.parm.capture.timeperframe.denominator = framerate;
    if (usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_S_PARM), &stream_params) < 0)
        ROS_ERROR("Video4Linux: cannot set desired framerate: %i fps (%i)", framerate,  errno);
    /* Final frame grabber setup */
    run_grabber(fmt.fmt.pix.sizeimage);

    image = reinterpret_cast<camera_image_t *>(calloc(1, sizeof(camera_image_t)));

    image->width = image_width;
    image->height = image_height;
    image->bytes_per_pixel = 3;  // corrected 11/10/15 (BYTES not BITS per pixel)

    image->image_size = image->width * image->height * image->bytes_per_pixel;
    image->is_new = 0;
    image->image = reinterpret_cast<char *>(calloc(image->image_size, sizeof(char)));
    memset(image->image, 0, image->image_size * sizeof(char));

    return true;
}

bool UsbCam::init_decoder()
{
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(58, 9, 100)
    avcodec_register_all();
#endif
    if(v4l_pixel_format == V4L2_PIX_FMT_MJPEG)
    {
        codec_id = AV_CODEC_ID_MJPEG;
        ROS_INFO("Initializing FFMPEG decoder for MJPEG compression");
    }
    else if(v4l_pixel_format == V4L2_PIX_FMT_H264)
    {
        codec_id = AV_CODEC_ID_H264;
        ROS_INFO("Initializing FFMPEG decoder for H.264 compression");
    }
    else
        return true;
    avcodec = const_cast<AVCodec*>(avcodec_find_decoder(codec_id));
    if(!avcodec)
    {
        ROS_ERROR("Cannot find FFMPEG decoder for %s", pixel_format_name.c_str());
        return false;
    }
    avcodec_context = avcodec_alloc_context3(avcodec);
    /* Suppress warnings of the following form:
     * [swscaler @ 0x############] deprecated pixel format used, make sure you did set range correctly
     * Or set this to AV_LOG_FATAL to additionally suppress occasional frame errors, e.g.:
     * [mjpeg @ 0x############] overread 4
     * [mjpeg @ 0x############] No JPEG data found in image
     * [ERROR] [##########.##########]: Error while decoding frame.
     */
    if(!full_ffmpeg_log)
        av_log_set_level(AV_LOG_ERROR);
    else
        av_log_set_level(AV_LOG_INFO);
#if LIBAVCODEC_VERSION_MAJOR < 55
    avframe_camera = avcodec_alloc_frame();
    avframe_rgb = avcodec_alloc_frame();
#else
    avframe_camera = av_frame_alloc();
    avframe_rgb = av_frame_alloc();
#endif
#if LIBAVCODEC_VERSION_MAJOR < 55
    avpicture_alloc(reinterpret_cast<AVPicture *>(avframe_rgb),
                    AV_PIX_FMT_RGB24,
                    image_width,
                    image_height);
#else
    /*av_image_alloc(reinterpret_cast<uint8_t **>(avframe_rgb),
                   0,
                   image_width,
                   image_height,
                   AV_PIX_FMT_RGB24,
                   1); */
    avframe_rgb->format = AV_PIX_FMT_RGB24;
    avframe_rgb->width = image_width;
    avframe_rgb->height = image_height;
    av_frame_get_buffer(avframe_rgb, 1);
#endif
    avcodec_context->codec_id = codec_id;
    avcodec_context->width = image_width;
    avcodec_context->height = image_height;
#if LIBAVCODEC_VERSION_MAJOR > 52
    if (color_format == COLOR_FORMAT_YUV422P)
        avcodec_context->pix_fmt = AV_PIX_FMT_YUV422P;
    else
        avcodec_context->pix_fmt = AV_PIX_FMT_YUV420P;
    avcodec_context->codec_type = AVMEDIA_TYPE_VIDEO;

    if(avcodec_context->codec_id == AV_CODEC_ID_MJPEG)
    {
        switch(avcodec_context->pix_fmt)
        {
        case AV_PIX_FMT_YUVJ420P:
            avcodec_context->pix_fmt = AV_PIX_FMT_YUV420P;
            avcodec_context->color_range = AVCOL_RANGE_JPEG;
            break;
        case AV_PIX_FMT_YUVJ422P:
            avcodec_context->pix_fmt = AV_PIX_FMT_YUV422P;
            avcodec_context->color_range = AVCOL_RANGE_JPEG;
            break;
        case AV_PIX_FMT_YUVJ444P:
            avcodec_context->pix_fmt = AV_PIX_FMT_YUV444P;
            avcodec_context->color_range = AVCOL_RANGE_JPEG;
            break;
        default:
            break;
        }
    }
#endif
#if LIBAVCODEC_VERSION_MAJOR < 55
    if(color_format == COLOR_FORMAT_YUV422P)
        avframe_camera_size = avpicture_get_size(AV_PIX_FMT_YUV422P, image_width, image_height);
    else
        avframe_camera_size = av_image_get_buffer_size(AV_PIX_FMT_YUV420P, image_width, image_height);
    avframe_rgb_size = avpicture_get_size(AV_PIX_FMT_RGB24, image_width, image_height);
#else
    if(color_format == COLOR_FORMAT_YUV422P)
        avframe_camera_size = av_image_get_buffer_size(AV_PIX_FMT_YUV422P, image_width, image_height, 1);
    else
        avframe_camera_size = av_image_get_buffer_size(AV_PIX_FMT_YUV420P, image_width, image_height, 1);
    avframe_rgb_size = av_image_get_buffer_size(AV_PIX_FMT_RGB24, image_width, image_height, 1);
#endif
    if(avcodec_open2(avcodec_context, avcodec, &avoptions) < 0)
    {
        ROS_ERROR("Cannot open FFMPEG decoder context");
        return false;
    }
    if((v4l_pixel_format == V4L2_PIX_FMT_MJPEG) || (v4l_pixel_format == V4L2_PIX_FMT_H264))     //  Setting up format converter between YUV and RGB from libswscale
        video_sws = sws_getContext(image_width,
                                   image_height,
                                   avcodec_context->pix_fmt,
                                   image_width,
                                   image_height,
                                   AV_PIX_FMT_RGB24,
                                   SWS_FAST_BILINEAR,
                                   nullptr,
                                   nullptr,
                                   nullptr);
    avpkt = av_packet_alloc();
    return true;
}

bool UsbCam::start_capture()
{
    if(streaming_status)
        return false;

    unsigned int i;
    enum v4l2_buf_type type;

    switch(io_method)
    {
    case IO_METHOD_READ:
        ROS_INFO("Capturing from block device, cancelling memory remap");
        break;
    case IO_METHOD_MMAP:
        for (i = 0; i < buffers_count; ++i)
        {
            struct v4l2_buffer buf;
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_QBUF), &buf) < 0)
            {
                ROS_ERROR("Video4linux: unable to configure stream (%i)", errno);
                return false;
            }
        }
        break;
    case IO_METHOD_USERPTR:
        for (i = 0; i < buffers_count; ++i)
        {
            struct v4l2_buffer buf;
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;
            buf.index = i;
            buf.m.userptr = reinterpret_cast<uint64_t>(buffers[i].start);
            buf.length = buffers[i].length;
            if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_QBUF), &buf) < 0)
            {
                ROS_ERROR("Video4linux: unable to configure stream (%i)", errno);
                return false;
            }
        }
        break;
    default:
        ROS_ERROR("Video4linux: attempt to start stream with unknown I/O method. Dropping request");
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (usb_cam::util::xioctl(file_dev, VIDIOC_STREAMON, &type) < 0)
    {
        ROS_ERROR("Video4linux: unable to start stream (%i)", errno);
        return false;
    }
    streaming_status = true;
    return true;
}

bool UsbCam::suspend()
{
    if(!streaming_status)
        return false;
    enum v4l2_buf_type type;
    streaming_status = false;
    switch(io_method)
    {
    case IO_METHOD_READ:
        return true;
    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (usb_cam::util::xioctl(file_dev, VIDIOC_STREAMOFF, &type) < 0)
        {
            ROS_ERROR("Video4linux: cannot stop the device properly (%i)", errno);
            return false;
        }
        return true;
    default:
        ROS_WARN("Attempt to stop streaming over unknown I/O channel");
        return false;
    }
}

void UsbCam::release_device()
{
    unsigned int i;
    switch(io_method)
    {
    case IO_METHOD_READ:
        free(buffers[0].start);
        break;
    case IO_METHOD_MMAP:
        for (i = 0; i < buffers_count; ++i)
        {
            if (munmap(buffers[i].start, buffers[i].length) < 0)
                ROS_ERROR("Video4linux: unable to deallocate frame buffers");
        }
        break;
    case IO_METHOD_USERPTR:
        for (i = 0; i < buffers_count; ++i)
            free(buffers[i].start);
        break;
    default:
        ROS_WARN("Attempt to free buffer for unknown I/O method");
    }
    free(buffers);
}

void UsbCam::close_handlers()
{
    int res = close(file_dev);
    file_dev = -1;
    if(res < 0)
        ROS_ERROR("Unable to close device handler properly");
}

camera_image_t *UsbCam::read_frame()
{
    if((image->width == 0) || (image->height == 0))
        return nullptr;

    fd_set fds;
    struct timeval tv;
    int r;
    FD_ZERO(&fds);
    FD_SET(file_dev, &fds);
    // Timeout
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    r = select(file_dev + 1, &fds, nullptr, nullptr, &tv);
    /* if the v4l2_buffer timestamp isn't available use this time, though
     * it may be 10s of milliseconds after the frame acquisition.
     * image->stamp = clock->now(); */
    timespec_get(&image->stamp, TIME_UTC);
    if(r < 0)
    {
        if(errno == EINTR)
            return nullptr;
        ROS_ERROR("Video4linux: frame mapping operation failed (%i)", errno);
    }
    else if(r == 0)
    {
        ROS_ERROR("Video4linux: frame mapping timeout (%i)", errno);
        return nullptr;
    }

    // Reading the actual frame
    struct v4l2_buffer buf;
    unsigned int i;
    int len;
    struct timespec stamp;
    int64_t buffer_time_s;
    switch(io_method)
    {
    case IO_METHOD_READ:
        len = read(file_dev, buffers[0].start, buffers[0].length);
        if(len < 0)
        {
            if(errno == EAGAIN)
                return nullptr;
            else if(errno == EIO){}
            else
            {
                ROS_ERROR("Block device read failure (%i)", errno);
                return nullptr;
            }
        }
        // Process image
        break;
    case IO_METHOD_MMAP:
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_DQBUF), &buf) < 0)
        {
            if(errno == EAGAIN)
                return nullptr;
            else if(errno == EIO){}
            else
            {
                ROS_ERROR("Memory mapping failure (%i)", errno);
                return nullptr;
            }
        }
        buffer_time_s = buf.timestamp.tv_sec + static_cast<int64_t>(round(buf.timestamp.tv_usec / 1000000.0));
        stamp.tv_sec = static_cast<time_t>(round(buffer_time_s)) + epoch_time_shift;
        stamp.tv_nsec = static_cast<int64_t>(buf.timestamp.tv_usec * 1000.0);
        assert(buf.index < buffers_count);
        len = buf.bytesused;
        // Process image
        if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_QBUF), &buf) < 0)
        {
            ROS_ERROR("Unable to exchange buffer with driver (%i)", errno);
            return nullptr;
        }
        image->stamp = stamp;
        break;
    case IO_METHOD_USERPTR:
        CLEAR(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        if (usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_DQBUF), &buf) < 0)
        {
            if(errno == EAGAIN)
                return nullptr;
            else if(errno == EIO){}
            else
            {
                ROS_ERROR("Unable to exchange poiner with driver (%i)", errno);
                return nullptr;
            }
        }
        buffer_time_s = buf.timestamp.tv_sec + static_cast<int64_t>(round(buf.timestamp.tv_usec / 1000000.0));

        stamp.tv_sec = static_cast<time_t>(round(buffer_time_s)) + epoch_time_shift;
        stamp.tv_nsec = static_cast<int64_t>(buf.timestamp.tv_usec / 1000.0);

        for(i = 0; i < buffers_count; ++i)
            if(buf.m.userptr == reinterpret_cast<uint64_t>(buffers[i].start) && buf.length == buffers[i].length)
                break;
        assert(i < buffers_count);
        len = buf.bytesused;
        // Process image
        if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_QBUF), &buf) < 0)
        {
            ROS_ERROR("Unable to exchange buffer with driver (%i)", errno);
            return nullptr;
        }
        image->stamp = stamp;
        break;
    default:
        ROS_WARN("Attempt to grab the frame via unknown I/O method (%i)", errno);
    }
    bool processing_result = false;
    if(io_method == IO_METHOD_READ)
        processing_result = process_image(buffers[0].start, len, image);
    else if(io_method == IO_METHOD_MMAP)
        processing_result = process_image(buffers[buf.index].start, len, image);
    else if(io_method == IO_METHOD_USERPTR)
        processing_result = process_image(reinterpret_cast<const void *>(buf.m.userptr), len, image);
    if(!processing_result)
    {
        ROS_ERROR("2D processing operation fault");
        return nullptr;
    }

    // Setting color table for grayscale image
    if(monochrome)
    {
        image->encoding = "mono8";
        image->step = image->width;
    }
    else
    {
        // TODO(lucasw) aren't there other encoding types?
        image->encoding = "rgb8";
        image->step = image->width * 3;
    }
    image->is_new = 1;
    return image;
}

bool UsbCam::decode_ffmpeg(const void *src, int len, camera_image_t *dest)
{
    char* MJPEG = const_cast<char *>(reinterpret_cast<const char *>(src));
    char* RGB = dest->image;
    static int got_picture = 1;
    // clear the picture
    memset(RGB, 0, avframe_rgb_size);
#if LIBAVCODEC_VERSION_MAJOR > 52
    av_new_packet(avpkt, len);
    av_packet_from_data(avpkt, reinterpret_cast<unsigned char*>(MJPEG), len);
    if(avcodec_send_packet(avcodec_context, avpkt) < 0)
    {
        ROS_ERROR("FFMPEG: error passing frame to decoder context");
        return false;
    }
#else
    avcodec_decode_video(avcodec_context, avframe_camera, &got_picture, (uint8_t *) MJPEG, len);
    if (!got_picture)
    {
        ROS_ERROR("FFMPEG: buffer empty: expected picture data");
        return;
    }
#endif
    if (avcodec_receive_frame(avcodec_context, avframe_camera) < 0)
    {
        ROS_ERROR("FFMPEG: error decoding frame");
        return false;
    }
    if (!got_picture)
    {
        ROS_ERROR("FFMPEG: MJPEG frame data expected, but was not received");
        return false;
    }
    int xsize = avcodec_context->width;
    int ysize = avcodec_context->height;

#if LIBAVCODEC_VERSION_MAJOR > 52
    int pic_size = av_image_get_buffer_size(avcodec_context->pix_fmt, xsize, ysize, 1);
#else
    // TODO(lucasw) avpicture_get_size corrupts the pix_fmt
    int pic_size = avpicture_get_size(avcodec_context->pix_fmt, xsize, ysize);
#endif
    // int pic_size = av_image_get_buffer_size(avcodec_context_->pix_fmt, xsize, ysize);
    if (pic_size != avframe_camera_size)
    {
        ROS_ERROR("FFMPEG: MJPEG output buffer size mismatch: %i (%i expected)", avframe_camera_size, pic_size);
        return false;
    }
    // YUV format conversion to RGB
    sws_scale(video_sws, avframe_camera->data, avframe_camera->linesize, 0, ysize, avframe_rgb->data, avframe_rgb->linesize);

#if LIBAVCODEC_VERSION_MAJOR > 52
    int size = av_image_copy_to_buffer(
        reinterpret_cast<uint8_t *>(RGB),
        avframe_rgb_size,
        avframe_rgb->data,
        avframe_rgb->linesize,
        AV_PIX_FMT_RGB24,
        xsize,
        ysize,
        1);
#else
    int size = avpicture_layout(
        reinterpret_cast<AVPicture *>(avframe_rgb), AV_PIX_FMT_RGB24,
        xsize, ysize, reinterpret_cast<uint8_t *>(RGB), avframe_rgb_size);
#endif
    if (size != avframe_rgb_size)
    {
        ROS_ERROR("FFMPEG: image layout mismatch: %i (%i expected)", size, avframe_rgb_size);
        return false;
    }
    return true;
}

bool UsbCam::process_image(const void *src, int len, camera_image_t *dest)
{
    bool result = false;
    if(v4l_pixel_format == V4L2_PIX_FMT_YUYV)
    {
        if(monochrome) // actually format V4L2_PIX_FMT_Y16, but usb_cam::utils::xioctl gets unhappy if you don't use the advertised type (yuyv)
            result = util::converters::MONO102MONO8(const_cast<char *>(reinterpret_cast<const char *>(src)), dest->image, dest->width * dest->height);
        else
            result = util::converters::YUYV2RGB(const_cast<char *>(reinterpret_cast<const char *>(src)), dest->image, dest->width * dest->height);
    }
    else if(v4l_pixel_format == V4L2_PIX_FMT_UYVY)
        result = util::converters::UYVY2RGB(const_cast<char *>(reinterpret_cast<const char *>(src)), dest->image, dest->width * dest->height);
    else if(v4l_pixel_format == V4L2_PIX_FMT_MJPEG || v4l_pixel_format == V4L2_PIX_FMT_H264)
        result = decode_ffmpeg(src, len, dest); // Internal conversion: condext-dependent
    else if(v4l_pixel_format == V4L2_PIX_FMT_RGB24 || v4l_pixel_format == V4L2_PIX_FMT_GREY)
        result = util::converters::COPY2RGB(const_cast<char *>(reinterpret_cast<const char *>(src)), dest->image, dest->width * dest->height);
    else if(v4l_pixel_format == V4L2_PIX_FMT_YUV420)
        result = util::converters::YUV4202RGB(const_cast<char *>(reinterpret_cast<const char *>(src)), dest->image, dest->width, dest->height);
    else if(v4l_pixel_format == V4L2_PIX_FMT_BGR24) // Direct copy for OpenCV
    {
        memcpy(dest, src, len);
        result = true;
    }
    return result;
}

std::vector<capture_format_t> &UsbCam::get_supported_formats()
{
    supported_formats.clear();
    struct v4l2_fmtdesc current_format;
    current_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    current_format.index = 0;
    for(current_format.index = 0;
         usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_ENUM_FMT), &current_format) == 0;
         current_format.index++)
    {
        struct v4l2_frmsizeenum current_size;
        current_size.index = 0;
        current_size.pixel_format = current_format.pixelformat;

        for(current_size.index = 0;
             usb_cam::util::xioctl(
                 file_dev, static_cast<int>(VIDIOC_ENUM_FRAMESIZES), &current_size) == 0;
             current_size.index++)
        {
            struct v4l2_frmivalenum current_interval;
            current_interval.index = 0;
            current_interval.pixel_format = current_size.pixel_format;
            current_interval.width = current_size.discrete.width;
            current_interval.height = current_size.discrete.height;
            for(current_interval.index = 0;
                 usb_cam::util::xioctl(
                     file_dev, static_cast<int>(VIDIOC_ENUM_FRAMEINTERVALS), &current_interval) == 0;
                 current_interval.index++)
            {
                if(current_interval.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
                    capture_format_t capture_format;
                    capture_format.format = current_format;
                    capture_format.size = current_size;
                    capture_format.interval = current_interval;
                    supported_formats.push_back(capture_format);
                }
            }  // interval loop
        }  // size loop
    }  // fmt loop
    return supported_formats;
}

void UsbCam::run_grabber(unsigned int &buffer_size)
{
    if(io_method == IO_METHOD_MMAP)
    {
        struct v4l2_requestbuffers req;
        CLEAR(req);
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_REQBUFS), &req) < 0)
        {
            if(errno == EINVAL)
                ROS_ERROR("Video4Linux: device '%s' does not support memory mapping (%i)", video_device_name.c_str(),  errno);
            else
                ROS_ERROR("Video4Linux: unable to start memory mapping (%i)", errno);
            return;
        }
        if(req.count < 2)
        {
            ROS_ERROR("Video4Linux: insufficient memory buffers number (%i)", req.count);
            return;
        }
        buffers = reinterpret_cast<buffer *>(calloc(req.count, sizeof(*buffers)));
        if(!buffers)
        {
            ROS_FATAL("Out of memory");
            return;
        }
        for (buffers_count = 0; buffers_count < req.count; ++buffers_count)
        {
            struct v4l2_buffer buf;

            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = buffers_count;

            if (usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_QUERYBUF), &buf) < 0)
            {
                ROS_ERROR("Video4Linux: unable to query buffer status (%i)", errno);
                return;
            }
            buffers[buffers_count].length = buf.length;
            buffers[buffers_count].start = mmap(NULL,
                                                buf.length,
                                                PROT_READ | PROT_WRITE,
                                                MAP_SHARED, file_dev,
                                                buf.m.offset);
            if (buffers[buffers_count].start == MAP_FAILED)
            {
                ROS_FATAL("Video4Linux: unable to allocate memory (%i)", errno);
                return;
            }
        }
    }
    else if(io_method == IO_METHOD_READ)
    {
        buffers = reinterpret_cast<buffer *>(calloc(1, sizeof(*buffers)));
        if (!buffers)
        {
            ROS_FATAL("Out of memory");
            return;
        }
        buffers[0].length = buffer_size;
        buffers[0].start = malloc(buffer_size);
        if (!buffers[0].start)
        {
            ROS_FATAL("Out of memory");
            return;
        }
    }
    else if(io_method == IO_METHOD_USERPTR)
    {
        struct v4l2_requestbuffers req;
        unsigned int page_size;
        page_size = getpagesize();
        buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);
        CLEAR(req);
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_USERPTR;
        if (usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_REQBUFS), &req) < 0)
        {
            if (errno == EINVAL)
            {
                ROS_FATAL("Video4Linux: device '%s' does not support USERPTR access mode", video_device_name.c_str());
                return;
            }
            else
            {
                ROS_FATAL("Video4Linux: device '%s' does not support streaming access", video_device_name.c_str());
                return;
            }
        }
        buffers = reinterpret_cast<buffer *>(calloc(4, sizeof(*buffers)));
        if (!buffers)
        {
            ROS_FATAL("Out of memory");
            return;
        }
        for (buffers_count = 0; buffers_count < 4; ++buffers_count)
        {
            buffers[buffers_count].length = buffer_size;
            buffers[buffers_count].start = memalign(page_size, buffer_size);
            if (!buffers[buffers_count].start)
            {
                ROS_FATAL("Out of memory");
                return;
            }
        }
    }
    else
    {
        ROS_ERROR("Cannot parse access mode for device '%s': '%s', system malfunction expected", video_device_name.c_str(), io_method_name.c_str());
        return;
    }
}

bool UsbCam::set_v4l_parameter(const std::string &param, const std::string &value)
{
    std::stringstream ss;
    ss << "v4l2-ctl --device=" << video_device_name << " -c " << param << "=" << value << " 2>&1";
    std::string cmd = ss.str();
    // capture the output
    std::string output;
    const int kBufferSize = 256;
    char buffer[kBufferSize];
    FILE * stream = popen(cmd.c_str(), "r");
    if(stream)
    {
        while (!feof(stream))
        {
            if(fgets(buffer, kBufferSize, stream) != NULL)
                output.append(buffer);
        }
        pclose(stream);
        // any output should be an error
        if (output.length() > 0)
        {
            ROS_ERROR("Video4linux: error setting camera parameter: '%s'", output.c_str());
            return false;
        }
    }
    else
    {
        ROS_ERROR("Video4linux: error running control command: '%s'", cmd.c_str());
        return false;
    }
    return true;
}

void UsbCam::adjust_camera()
{
    ROS_INFO("Setting up auxiliary camera parameters");
    // Autofocus
    if(autofocus)
    {
        struct v4l2_queryctrl queryctrl;
        struct v4l2_ext_control control;

        memset(&queryctrl, 0, sizeof(queryctrl));
        queryctrl.id = V4L2_CID_FOCUS_AUTO;

        if (usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_QUERYCTRL), &queryctrl) < 0)
        {
            if (errno != EINVAL)
                ROS_ERROR("Video4linux: cannot query auxiliary control (FOCUS_AUTO, %i)", errno);
            else
                ROS_ERROR("Video4linux: auto focus not supported (%i)", errno);
        }
        else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
            ROS_ERROR("Video4linux: auto focus not supported (%i)", errno);
        else
        {
            memset(&control, 0, sizeof(control));
            control.id = V4L2_CID_FOCUS_AUTO;
            control.value = 1;

            if (usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_S_CTRL), &control) < 0)
                ROS_ERROR("Video4linux: auxiliary control not supported (%i)", errno);
        }
        set_v4l_parameter("focus_auto", 1);
    }
    else
    {
        set_v4l_parameter("focus_auto", 0);
        if(focus >= 0)
            set_v4l_parameter("focus_absolute", focus);
    }
    // Autoexposure
    if(autoexposure)
        set_v4l_parameter("exposure_auto", 1);
    else
    {
        set_v4l_parameter("exposure_auto", 0);
        set_v4l_parameter("exposure_absolute", exposure);
    }
    // Auto white balance
    if(auto_white_balance)
        set_v4l_parameter("white_balance_temperature_auto", 1);
    else
    {
        set_v4l_parameter("white_balance_temperature_auto", 0);
        set_v4l_parameter("white_balance_temperature", white_balance);
    }
    // Brightness
    if(brightness >= 0)
        set_v4l_parameter("brightness", brightness);
    // Contrast
    if(contrast >= 0)
        set_v4l_parameter("contrast", contrast);
    // Saturation
    if(saturation >= 0)
        set_v4l_parameter("saturation", saturation);
    // Sharpness
    if(sharpness >= 0)
        set_v4l_parameter("sharpness", sharpness);
    // Gain
    if(gain >= 0)
        set_v4l_parameter("gain", gain);
}

UsbCam::UsbCam():
    node("~"),
    _img_msg(),
    _image_transport(node)
{
    img_msg = &_img_msg;
    image_transport = &_image_transport;
    /* Loading parameters */
    // Mandatory parameters are defined with getParam(), otherwise with param<>()
    node.getParam("video_device", video_device_name);
    node.getParam("io_method", io_method_name);
    node.getParam("pixel_format", pixel_format_name);
    node.getParam("color_format", color_format_name);
    node.param<bool>("create_suspended", create_suspended, false);
    node.param<bool>("full_ffmpeg_log", full_ffmpeg_log, false);
    node.getParam("camera_name", camera_name);
    node.getParam("camera_frame_id", camera_frame_id);
    node.param<std::string>("camera_transport_suffix", camera_transport_suffix, "image_raw");
    node.param<std::string>("camera_info_url", camera_info_url, "");
    node.getParam("image_width", image_width);
    node.getParam("image_height", image_height);
    node.getParam("framerate", framerate);
    node.param<int>("exposure", exposure, 100);
    node.param<int>("brightness", brightness, -1);
    node.param<int>("contrast", contrast, -1);
    node.param<int>("saturation", saturation, -1);
    node.param<int>("sharpness", sharpness, -1);
    node.param<int>("focus", focus, -1);
    node.param<int>("white_balance", white_balance, 4000);
    node.param<int>("gain", gain, -1);
    node.param<bool>("autofocus", autofocus, false);
    node.param<bool>("autoexposure", autoexposure, true);
    node.param<bool>("auto_white_balance", auto_white_balance, false);
    node.param<std::string>("start_service_name", _service_start_name, "start_capture");
    node.param<std::string>("stop_service_name", _service_stop_name, "stop_capture");

    // Advertising camera
    ROS_INFO("Initializing ROS V4L USB camera '%s' (%s) at %dx%d via %s (%s) at %i FPS",
             camera_name.c_str(),
             video_device_name.c_str(),
             image_width,
             image_height,
             io_method_name.c_str(),
             pixel_format_name.c_str(),
             framerate);
    _image_pub = image_transport->advertiseCamera(camera_transport_suffix, 1);
    image_pub = &_image_pub;
    camera_info = new camera_info_manager::CameraInfoManager(node, camera_name, camera_info_url);
    img_msg->header.frame_id = camera_frame_id;
    if(!camera_info->isCalibrated())
    {
        camera_info->setCameraName(video_device_name);
        sensor_msgs::CameraInfo camera_info_msg;
        camera_info_msg.header.frame_id = img_msg->header.frame_id;
        camera_info_msg.width = image_width;
        camera_info_msg.height = image_height;
        camera_info->setCameraInfo(camera_info_msg);
    }

    // Resolving I/O method name tables
    io_method = util::converters::io_method_from_string(io_method_name);
    if(io_method == IO_METHOD_UNKNOWN)
    {
        ROS_FATAL("Unknown IO method '%s'", io_method_name.c_str());
        node.shutdown();
        return;
    }
    pixel_format = util::converters::pixel_format_from_string(pixel_format_name);
    if(pixel_format == PIXEL_FORMAT_UNKNOWN)
    {
        ROS_FATAL("Unknown pixel format '%s'", pixel_format_name.c_str());
        node.shutdown();
        return;
    }
    color_format = util::converters::color_format_from_string(color_format_name);
    if(color_format == COLOR_FORMAT_UNKNOWN)
    {
        ROS_FATAL("Unknown color format '%s'", color_format_name.c_str());
        node.shutdown();
        return;
    }
    v4l_pixel_format = util::converters::v4l_pixel_format_from_pixel_format(pixel_format, monochrome);
    if(v4l_pixel_format == UINT_MAX)
    {
        ROS_FATAL("Error in conversion between FFMPEG and Video4Linux pixel format constant '%s'", pixel_format_name.c_str());
        node.shutdown();
        return;
    }

    /* Initializing decoder */
    if(!init_decoder())
    {
        ROS_FATAL("Unable to initialize FFMPEG decoder");
        node.shutdown();
        return;
    }

    /* Advertising services */
    ROS_INFO("Advertising std_srvs::Empty start service under name '%s'", _service_start_name.c_str());
    _service_start = node.advertiseService(_service_start_name, &UsbCam::service_start_callback);
    service_start = &_service_start;
    ROS_INFO("Advertising std_srvs::Empty suspension service under name '%s'", _service_stop_name.c_str());
    _service_stop = node.advertiseService(_service_stop_name, &UsbCam::service_stop_callback);
    service_stop = &_service_stop;
    ROS_INFO("Advertising std_srvs::Trigger supported formats information service under name 'supported_formats'");
    _service_supported_formats = node.advertiseService("supported_formats", &UsbCam::service_supported_formats_callback);
    service_supported_formats = &_service_supported_formats;

    /* All parameters set, running frame grabber */
    if(!start())
    {
        ROS_FATAL("Error starting device");
        node.shutdown();
        return;
    }
    adjust_camera();
    // Creating timer
    ros::Duration frame_period(1.f / static_cast<float>(framerate));
    _frame_timer = node.createTimer(frame_period, &UsbCam::frame_timer_callback, false, true);
    frame_timer = &_frame_timer;
    // Running capture engine
    if(!create_suspended)
        if(!start_capture())
        {
            ROS_FATAL("Error starting capture device");
            node.shutdown();
            return;
        }
}

void UsbCam::frame_timer_callback(const ros::TimerEvent &event)
{
    if(streaming_status)
    {
        camera_image_t* new_image = read_frame();
        if(new_image == nullptr)
        {
            ROS_ERROR("Video4linux: frame grabber failed");
            return;
        }
        img_msg->header.stamp.sec = new_image->stamp.tv_sec;
        img_msg->header.stamp.nsec = new_image->stamp.tv_nsec;
        if (img_msg->data.size() != static_cast<size_t>(new_image->step * new_image->height))
        {
            img_msg->width = new_image->width;
            img_msg->height = new_image->height;
            img_msg->encoding = new_image->encoding;
            img_msg->step = new_image->step;
            img_msg->data.resize(new_image->step * new_image->height);
        }
        // Fill in image data
        memcpy(&img_msg->data[0], new_image->image, img_msg->data.size());
        auto ci = std::make_unique<sensor_msgs::CameraInfo>(camera_info->getCameraInfo());
        ci->header = img_msg->header;
        image_pub->publish((*img_msg), (*ci));
    }
}

UsbCam::~UsbCam()
{
    suspend();
    release_device();
    close_handlers();

    av_packet_free(&avpkt);
    if(video_sws)
        sws_freeContext(video_sws);
    video_sws = nullptr;
    if(avcodec_context)
    {
        avcodec_close(avcodec_context);
        av_free(avcodec_context);
        avcodec_context = nullptr;
    }
    if(avframe_camera)
        av_free(avframe_camera);
    avframe_camera = nullptr;
    if(avframe_rgb)
        av_free(avframe_rgb);
    avframe_rgb = nullptr;
    if(image)
        free(image);
    image = nullptr;
    delete camera_info;
}

usb_cam::UsbCam &usb_cam::UsbCam::Instance()
{
    static UsbCam instance;
    return instance;
}

