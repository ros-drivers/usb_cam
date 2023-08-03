#include <cstdio>
#include <linux/videodev2.h>
#include <sstream>
#include <string>

#include "usb_cam/camera_driver.h"
#include "usb_cam/converters.h"
#include "usb_cam/types.h"

using namespace usb_cam;

/* STATIC DATA INITIALIZERS */
/* V4L/HARDWARE */
io_method_t AbstractV4LUSBCam::io_method = io_method_t::IO_METHOD_MMAP; // io_
pixel_format_t AbstractV4LUSBCam::pixel_format = PIXEL_FORMAT_UNKNOWN;
color_format_t AbstractV4LUSBCam::color_format = COLOR_FORMAT_UNKNOWN;
bool AbstractV4LUSBCam::monochrome = false;
int AbstractV4LUSBCam::file_dev = -1;
const time_t AbstractV4LUSBCam::epoch_time_shift_us = util::get_epoch_time_shift_us();

/* FFMPEG */
bool AbstractV4LUSBCam::full_ffmpeg_log = false;
buffer* AbstractV4LUSBCam::buffers = nullptr;
unsigned int AbstractV4LUSBCam::buffers_count = 0; // n_buffers_
AVFrame* AbstractV4LUSBCam::avframe_camera = nullptr;
AVFrame* AbstractV4LUSBCam::avframe_rgb = nullptr;
AVPacket* AbstractV4LUSBCam::avpkt = nullptr;
AVCodec* AbstractV4LUSBCam::avcodec = nullptr;
AVCodecID AbstractV4LUSBCam::codec_id = AV_CODEC_ID_NONE;
AVDictionary* AbstractV4LUSBCam::avoptions = nullptr;
AVCodecContext* AbstractV4LUSBCam::avcodec_context = nullptr;
int AbstractV4LUSBCam::avframe_camera_size = 0;
int AbstractV4LUSBCam::avframe_rgb_size = 0;
struct SwsContext* AbstractV4LUSBCam::video_sws = nullptr;
camera_image_t* AbstractV4LUSBCam::image = nullptr;
bool AbstractV4LUSBCam::capturing = false;
std::vector<capture_format_t> AbstractV4LUSBCam::supported_formats = std::vector<capture_format_t>();

/* V4L camera parameters */
bool AbstractV4LUSBCam::streaming_status = false;
std::string AbstractV4LUSBCam::video_device_name = "/dev/video0";
std::string AbstractV4LUSBCam::io_method_name = "mmap";
std::string AbstractV4LUSBCam::pixel_format_name = "uyvy";
unsigned int AbstractV4LUSBCam::v4l_pixel_format = V4L2_PIX_FMT_UYVY;
std::string AbstractV4LUSBCam::color_format_name = "yuv422p";
int AbstractV4LUSBCam::image_width = 320;
int AbstractV4LUSBCam::image_height = 240;
int AbstractV4LUSBCam::framerate = 10;
std::vector<camera_control_t> AbstractV4LUSBCam::controls = std::vector<camera_control_t>();
std::set<std::string> AbstractV4LUSBCam::ignore_controls = std::set<std::string>();


bool AbstractV4LUSBCam::init()
{
    // Resolving I/O method name tables
    io_method = util::converters::io_method_from_string(io_method_name);
    if(io_method == IO_METHOD_UNKNOWN)
    {
        printf("Unknown IO method '%s'\n", io_method_name.c_str());
        return false;
    }
    pixel_format = util::converters::pixel_format_from_string(pixel_format_name);
    if(pixel_format == PIXEL_FORMAT_UNKNOWN)
    {
        printf("Unknown pixel format '%s'\n", pixel_format_name.c_str());
        return false;
    }
    color_format = util::converters::color_format_from_string(color_format_name);
    if(color_format == COLOR_FORMAT_UNKNOWN)
    {
        printf("Unknown color format '%s'\n", color_format_name.c_str());
        return false;
    }
    v4l_pixel_format = util::converters::v4l_pixel_format_from_pixel_format(pixel_format, monochrome);
    if(v4l_pixel_format == UINT_MAX)
    {
        printf("Error in conversion between FFMPEG and Video4Linux pixel format constant '%s'\n", pixel_format_name.c_str());
        return false;
    }

    /* Initializing decoder */
    if(!init_decoder())
    {
        printf("Unable to initialize FFMPEG decoder\n");
        return false;
    }
    return true;
}

bool AbstractV4LUSBCam::start()
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
    printf("Opening streaming device %s\n", video_device_name.c_str());
    if(stat(video_device_name.c_str(), &st) < 0)
    {
        printf("Cannot identify device by name '%s' (%i)\n", video_device_name.c_str(), errno);
        return false;
    }
    if(!S_ISCHR(st.st_mode))
    {
        printf("'%s' is not a proper V4L device (%i)\n", video_device_name.c_str(), errno);
        return false;
    }
    file_dev = open(video_device_name.c_str(),
                    O_RDWR|O_NONBLOCK,
                    0);
    if(file_dev < 0)
    {
        printf("Cannot create a file handler for V4L device '%s' (%i)\n", video_device_name.c_str(), errno);
        return false;
    }

    /* Initializing V4L capture pipeline */
    if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_QUERYCAP), &cap) < 0)
    {
        if(errno == EINVAL)
            printf("File handler created for V4L-incompatible device '%s' (%i)\n", video_device_name.c_str(), errno);
        else
            printf("Cannot query capabilities from V4L device '%s' (%i)\n", video_device_name.c_str(), errno);
        return false;
    }
    if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        printf("V4L device '%s' does not support capture mode (%i)\n", video_device_name.c_str(), errno);
        return false;
    }
    switch(io_method)
    {
    case IO_METHOD_READ:
        if(!(cap.capabilities & V4L2_CAP_READWRITE))
        {
            printf("Device '%s' does not support '%s' access method (read/write error)\n", video_device_name.c_str(), io_method_name.c_str());
            return false;
        }
        break;
    case io_method_t::IO_METHOD_MMAP:
    case io_method_t::IO_METHOD_USERPTR:
        if(!(cap.capabilities & V4L2_CAP_STREAMING))
        {
            printf("Device '%s' does not support '%s' access method (streaming error)\n", video_device_name.c_str(), io_method_name.c_str());
            return false;
        }
        break;
    default:
        printf("Cannot parse access mode for device '%s': '%s', system malfunction expected\n", video_device_name.c_str(), io_method_name.c_str());
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
                printf("Video4Linux: CROP  mode is not supported\n");
            else
                printf("Video4Linux: IOCTL is not supported\n");
        }
    }
    else
        printf("Video4Linux: internal error occurred, hoping for device fallback\n");
    CLEAR(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = image_width;
    fmt.fmt.pix.height = image_height;
    fmt.fmt.pix.pixelformat = v4l_pixel_format;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_S_FMT), &fmt) < 0)
    {
        printf("Cannot set pixel format '%s' (%u)\n", pixel_format_name.c_str(), v4l_pixel_format);
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
        printf("Cannot set stream parameters (%i)\n", errno);
        return false;
    }
    if(!stream_params.parm.capture.capability && V4L2_CAP_TIMEPERFRAME)
        printf("Video4Linux: V4L2_CAP_TIMEPERFRAME not supported\n");
    // TODO(lucasw) need to get list of valid numerator/denominator pairs
    // and match closest to what user put in.
    stream_params.parm.capture.timeperframe.numerator = 1;
    stream_params.parm.capture.timeperframe.denominator = framerate;
    if (usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_S_PARM), &stream_params) < 0)
        printf("Video4Linux: cannot set desired framerate: %i fps (%i)\n", framerate,  errno);
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

AbstractV4LUSBCam::~AbstractV4LUSBCam()
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
}

bool AbstractV4LUSBCam::init_decoder()
{
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(58, 9, 100)
    avcodec_register_all();
#endif
    if(v4l_pixel_format == V4L2_PIX_FMT_MJPEG)
    {
        codec_id = AV_CODEC_ID_MJPEG;
        printf("Initializing FFMPEG decoder for MJPEG compression\n");
    }
    else if(v4l_pixel_format == V4L2_PIX_FMT_H264)
    {
        codec_id = AV_CODEC_ID_H264;
        printf("Initializing FFMPEG decoder for H.264 compression\n");
    }
    else
        return true;
    avcodec = const_cast<AVCodec*>(avcodec_find_decoder(codec_id));
    if(!avcodec)
    {
        printf("Cannot find FFMPEG decoder for %s\n", pixel_format_name.c_str());
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
        printf("Cannot open FFMPEG decoder context\n");
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

bool AbstractV4LUSBCam::start_capture()
{
    if(streaming_status)
        return false;

    unsigned int i;
    enum v4l2_buf_type type;

    switch(io_method)
    {
    case IO_METHOD_READ:
        printf("Capturing from block device, cancelling memory remap\n");
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
                printf("Video4linux: unable to configure stream (%i)\n", errno);
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
                printf("Video4linux: unable to configure stream (%i)\n", errno);
                return false;
            }
        }
        break;
    default:
        printf("Video4linux: attempt to start stream with unknown I/O method. Dropping request\n");
    }
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (usb_cam::util::xioctl(file_dev, VIDIOC_STREAMON, &type) < 0)
    {
        printf("Video4linux: unable to start stream (%i)\n", errno);
        return false;
    }
    streaming_status = true;
    return true;
}

bool AbstractV4LUSBCam::suspend()
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
            printf("Video4linux: cannot stop the device properly (%i)\n", errno);
            return false;
        }
        return true;
    default:
        printf("Attempt to stop streaming over unknown I/O channel\n");
        return false;
    }
}

void AbstractV4LUSBCam::release_device()
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
                printf("Video4linux: unable to deallocate frame buffers\n");
        }
        break;
    case IO_METHOD_USERPTR:
        for (i = 0; i < buffers_count; ++i)
            free(buffers[i].start);
        break;
    default:
        printf("Attempt to free buffer for unknown I/O method\n");
    }
    free(buffers);
}

void AbstractV4LUSBCam::close_handlers()
{
    int res = close(file_dev);
    file_dev = -1;
    if(res < 0)
        printf("Unable to close device handler properly\n");
}

AbstractV4LUSBCam::AbstractV4LUSBCam()
{
}

camera_image_t *AbstractV4LUSBCam::read_frame()
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
        printf("Video4linux: frame mapping operation failed (%i)\n", errno);
    }
    else if(r == 0)
    {
        printf("Video4linux: frame mapping timeout (%i)\n", errno);
        return nullptr;
    }

    // Reading the actual frame
    struct v4l2_buffer buf;
    unsigned int i;
    int len;
    struct timespec stamp;
    int64_t buffer_time_us;
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
                printf("Block device read failure (%i)\n", errno);
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
                printf("Memory mapping failure (%i)\n", errno);
                return nullptr;
            }
        }
        image->stamp = util::calc_img_timestamp(buf.timestamp, epoch_time_shift_us);
        timespec_get(&image->stamp, TIME_UTC);
        assert(buf.index < buffers_count);
        len = buf.bytesused;
        // Process image
        if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_QBUF), &buf) < 0)
        {
            printf("Unable to exchange buffer with driver (%i)\n", errno);
            return nullptr;
        } 
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
                printf("Unable to exchange poiner with driver (%i)\n", errno);
                return nullptr;
            }
        }
        image->stamp = util::calc_img_timestamp(buf.timestamp, epoch_time_shift_us);
        timespec_get(&image->stamp, TIME_UTC);

        for(i = 0; i < buffers_count; ++i)
            if(buf.m.userptr == reinterpret_cast<uint64_t>(buffers[i].start) && buf.length == buffers[i].length)
                break;
        assert(i < buffers_count);
        len = buf.bytesused;
        // Process image
        if(usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_QBUF), &buf) < 0)
        {
            printf("Unable to exchange buffer with driver (%i)\n", errno);
            return nullptr;
        } 
        break;
    default:
        printf("Attempt to grab the frame via unknown I/O method (%i)\n", errno);
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
        printf("2D processing operation fault\n");
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

void AbstractV4LUSBCam::run_grabber(unsigned int &buffer_size)
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
                printf("Video4Linux: device '%s' does not support memory mapping (%i)\n", video_device_name.c_str(),  errno);
            else
                printf("Video4Linux: unable to start memory mapping (%i)\n", errno);
            return;
        }
        if(req.count < 2)
        {
            printf("Video4Linux: insufficient memory buffers number (%i)\n", req.count);
            return;
        }
        buffers = reinterpret_cast<buffer *>(calloc(req.count, sizeof(*buffers)));
        if(!buffers)
        {
            printf("Out of memory\n");
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
                printf("Video4Linux: unable to query buffer status (%i)\n", errno);
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
                printf("Video4Linux: unable to allocate memory (%i)\n", errno);
                return;
            }
        }
    }
    else if(io_method == IO_METHOD_READ)
    {
        buffers = reinterpret_cast<buffer *>(calloc(1, sizeof(*buffers)));
        if (!buffers)
        {
            printf("Out of memory\n");
            return;
        }
        buffers[0].length = buffer_size;
        buffers[0].start = malloc(buffer_size);
        if (!buffers[0].start)
        {
            printf("Out of memory\n");
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
                printf("Video4Linux: device '%s' does not support USERPTR access mode\n", video_device_name.c_str());
                return;
            }
            else
            {
                printf("Video4Linux: device '%s' does not support streaming access\n", video_device_name.c_str());
                return;
            }
        }
        buffers = reinterpret_cast<buffer *>(calloc(4, sizeof(*buffers)));
        if (!buffers)
        {
            printf("Out of memory\n");
            return;
        }
        for (buffers_count = 0; buffers_count < 4; ++buffers_count)
        {
            buffers[buffers_count].length = buffer_size;
            buffers[buffers_count].start = memalign(page_size, buffer_size);
            if (!buffers[buffers_count].start)
            {
                printf("Out of memory\n");
                return;
            }
        }
    }
    else
    {
        printf("Cannot parse access mode for device '%s': '%s', system malfunction expected\n", video_device_name.c_str(), io_method_name.c_str());
        return;
    }
}

bool AbstractV4LUSBCam::decode_ffmpeg(const void *src, int len, camera_image_t *dest)
{
    char* MJPEG = const_cast<char *>(reinterpret_cast<const char *>(src));
    char* RGB = dest->image;
    static int got_picture = 1;
    // clear the picture
    memset(RGB, 0, avframe_rgb_size);
#if LIBAVCODEC_VERSION_MAJOR > 52
    av_init_packet(avpkt);
    av_packet_from_data(avpkt, reinterpret_cast<unsigned char*>(MJPEG), len);
    if(avcodec_send_packet(avcodec_context, avpkt) < 0)
    {
        printf("FFMPEG: error passing frame to decoder context\n");
        return false;
    }
#else
    avcodec_decode_video(avcodec_context, avframe_camera, &got_picture, (uint8_t *) MJPEG, len);
    if (!got_picture)
    {
        printf("FFMPEG: buffer empty: expected picture data\n");
        return;
    }
#endif
    if (avcodec_receive_frame(avcodec_context, avframe_camera) < 0)
    {
        printf("FFMPEG: error decoding frame\n");
        return false;
    }
    if (!got_picture)
    {
        printf("FFMPEG: MJPEG frame data expected, but was not received\n");
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
        printf("FFMPEG: MJPEG output buffer size mismatch: %i (%i expected)\n", avframe_camera_size, pic_size);
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
        printf("FFMPEG: image layout mismatch: %i (%i expected)\n", size, avframe_rgb_size);
        return false;
    }
    return true;
}

std::vector<capture_format_t> &AbstractV4LUSBCam::get_supported_formats()
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

bool AbstractV4LUSBCam::process_image(const void *src, int len, camera_image_t *dest)
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
        memcpy(dest->image, src, len);
        result = true;
    }
    return result;
}

bool AbstractV4LUSBCam::set_v4l_parameter(const std::string &param, const std::string &value)
{
    std::stringstream ss;
    ss << "v4l2-ctl --device=" << video_device_name << " -c " << param << "=" << value << " 2>&1";
    std::string cmd = ss.str();
    // printf("%s\n", cmd.c_str());
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
            printf("Video4linux: error setting camera parameter: '%s'\n", output.c_str());
            return false;
        }
    }
    else
    {
        printf("Video4linux: error running control command: '%s'\n", cmd.c_str());
        return false;
    }
    return true;
}

void AbstractV4LUSBCam::v4l_query_controls()
{
    // https://gist.github.com/tugstugi/2627647
    // https://www.kernel.org/doc/html/v4.8/media/uapi/v4l/extended-controls.html
    printf("Video4linux: Querying V4L2 driver for available controls (register base 0x%X, 0..99)\n", V4L2_CID_BASE);
    struct v4l2_queryctrl ctrl;
    struct v4l2_querymenu menu;
    memset (&ctrl, 0, sizeof (ctrl));
    memset (&menu, 0, sizeof (menu));
    std::vector<std::string> disabled_controls;
    // std::vector<std::string> ignored_controls;
    ctrl.id = V4L2_CID_BASE;
    while(ioctl(file_dev, VIDIOC_QUERYCTRL, &ctrl) == 0)
    {
        camera_control_t control;
        std::stringstream description;
        if(ctrl.flags & V4L2_CTRL_FLAG_DISABLED)
        {
            disabled_controls.push_back(util::converters::v4l_control_name_to_param_name(reinterpret_cast<char*>(ctrl.name)));
        }
        else
        {
            control.name = util::converters::v4l_control_name_to_param_name(reinterpret_cast<char*>(ctrl.name));
            control.type = static_cast<v4l2_ctrl_type>(ctrl.type);
            control.value = std::to_string(ctrl.default_value);
            description << std::string(reinterpret_cast<char*>(ctrl.name)) << ", min = "
                        << std::to_string(ctrl.minimum) << ", max = "
                        << std::to_string(ctrl.maximum) << ", step = "
                        << std::to_string(ctrl.step) << ", flags = 0x" << std::hex << ctrl.flags << std::dec;
            if(ctrl.type == V4L2_CTRL_TYPE_MENU)
            {
                menu.id = ctrl.id;
                description << " [ ";
                for(menu.index = ctrl.minimum; menu.index <= ctrl.maximum; menu.index++)
                {
                    if(ioctl(file_dev, VIDIOC_QUERYMENU, &menu) == 0)
                        description << menu.index << ": " << std::string(reinterpret_cast<char*>(menu.name)) << " ";
                }
                description << "]";
            }
            control.description = description.str();
            controls.push_back(control);
        }
        //std::cout << util::converters::v4l_control_name_to_param_name(reinterpret_cast<char*>(ctrl.name)) << std::endl;

        ctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }
    if(!disabled_controls.empty())
    {
        std::cout << "Disabled controls: ";
        for(auto dc: disabled_controls)
            std::cout << dc << " ";
        std::cout << std::endl;
    }
    printf("Sorting control names:\n");
    if(!controls.empty())
    {
        for(auto c = controls.begin(); c != controls.end(); c++)
        {
            if(c->name.find("auto") != std::string::npos)
            {
                camera_control_t tc = (*c);
                controls.erase(c);
                controls.insert(controls.begin(), tc);
            }
        }
    }
    for(auto c: controls)
        printf("\t%s\n", c.name.c_str());
}

void AbstractV4LUSBCam::adjust_camera()
{
    printf("Video4linux: Setting up auxiliary camera parameters\n");
    if(controls.empty())
    {
        printf("Video4linux: camera controls was not queried properly, please call v4l_query_controls() before!\n");
        return;
    }
    for(auto control: controls)
    {
        if(ignore_controls.find(control.name) == ignore_controls.end())
            if(!set_v4l_parameter(control.name, control.value))
                printf("Video4linux: cannot set V4L control %s\n", control.name.c_str());
    }
    /* REPLACED WITH DYNAMICALLY QUERIED PARAMETERS
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
                printf("Video4linux: cannot query auxiliary control (FOCUS_AUTO, %i)\n", errno);
            else
                printf("Video4linux: auto focus not supported (%i)\n", errno);
        }
        else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
            printf("Video4linux: auto focus not supported (%i)\n", errno);
        else
        {
            memset(&control, 0, sizeof(control));
            control.id = V4L2_CID_FOCUS_AUTO;
            control.value = 1;

            if (usb_cam::util::xioctl(file_dev, static_cast<int>(VIDIOC_S_CTRL), &control) < 0)
                printf("Video4linux: auxiliary control not supported (%i)\n", errno);
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
    */
}
