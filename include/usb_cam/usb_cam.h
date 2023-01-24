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
#ifndef USB_CAM_USB_CAM_H
#define USB_CAM_USB_CAM_H

#include "ros/service_server.h"
#include <string>
extern "C"
{
// Legacy FFMPEG workaround
#include <libavcodec/version.h>
#if LIBAVCODEC_VERSION_MAJOR < 55
    #define AV_CODEC_ID_MJPEG CODEC_ID_MJPEG
#endif
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#define __STDC_CONSTANT_MACROS  // Required for libavutil
#include <libavutil/imgutils.h>
#include <libavutil/frame.h>
#include <libavutil/mem.h>
}


#include <fcntl.h>  // for O_* constants
#include <unistd.h>  // for getpagesize()
#include <malloc.h>  // for memalign
#include <sys/mman.h>  // for mmap
#include <sys/stat.h>  // for stat

#include "opencv2/imgproc.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include "usb_cam/types.h"
#include "usb_cam/util.h"
#include "usb_cam/converters.h"

namespace usb_cam
{

class UsbCam
{
private:
    /* SINGLETON */
    explicit UsbCam();
    virtual ~UsbCam();
    UsbCam(const UsbCam& root) = delete;
    UsbCam operator=(const UsbCam& root) = delete;
protected:
    /* V4L/HARDWARE */
    static io_method_t io_method; // io_
    static pixel_format_t pixel_format;
    static unsigned int v4l_pixel_format;
    static color_format_t color_format;
    static bool monochrome;
    static int file_dev; // fd_
    static const time_t epoch_time_shift;
    static bool create_suspended;

    /* FFMPEG */
    static bool full_ffmpeg_log;
    static buffer* buffers;
    static unsigned int buffers_count; // n_buffers_
    static AVFrame * avframe_camera;
    static AVFrame * avframe_rgb;
    static AVPacket* avpkt;
    static AVCodec * avcodec;
    static AVCodecID codec_id;
    static AVDictionary * avoptions;
    static AVCodecContext * avcodec_context;
    static int avframe_camera_size;
    static int avframe_rgb_size;
    static struct SwsContext * video_sws;
    camera_image_t _image;
    static camera_image_t * image;
    static bool capturing;
    static std::vector<capture_format_t> supported_formats;

    /* ROS */
    ros::NodeHandle node;
    ros::Timer _frame_timer;
    static ros::Timer* frame_timer;
    static void frame_timer_callback(const ros::TimerEvent& event);

    /* Image stream publisher */
    sensor_msgs::Image _img_msg; // img_
    static sensor_msgs::Image* img_msg;
    image_transport::CameraPublisher _image_pub;
    static image_transport::CameraPublisher* image_pub;
    static camera_info_manager::CameraInfoManager* camera_info; // cinfo_
    image_transport::ImageTransport _image_transport; // it_
    static image_transport::ImageTransport* image_transport;

    /* Service servers */
    std::string _service_start_name;
    ros::ServiceServer _service_start;
    static ros::ServiceServer* service_start;
    static bool service_start_callback(std_srvs::Empty::Request& request,
                                       std_srvs::Empty::Response& response);
    std::string _service_stop_name;
    ros::ServiceServer _service_stop;
    static ros::ServiceServer* service_stop;
    static bool service_stop_callback(std_srvs::Empty::Request& request,
                                      std_srvs::Empty::Response& response);
    ros::ServiceServer _service_supported_formats;
    static ros::ServiceServer* service_supported_formats;
    static bool service_supported_formats_callback(std_srvs::Trigger::Request& request,
                                                   std_srvs::Trigger::Response& response);

    /* V4L camera parameters */
    static bool streaming_status;
    static std::string video_device_name;
    static std::string io_method_name;
    static std::string pixel_format_name;
    static std::string color_format_name;
    static std::string camera_name;
    static std::string camera_frame_id;
    static std::string camera_transport_suffix;
    static std::string camera_info_url;
    static int image_width;
    static int image_height;
    static int framerate;
    static int exposure;
    static int brightness;
    static int contrast;
    static int saturation;
    static int sharpness;
    static int focus;
    static int white_balance;
    static int gain;
    static bool autofocus;
    static bool autoexposure;
    static bool auto_white_balance;

    /* Private functions */
    static bool start();
    static bool init_decoder();
    static void run_grabber(unsigned int& buffer_size);
    static bool set_v4l_parameter(const std::string & param, const std::string & value);
    static inline bool set_v4l_parameter(const std::string & param, int value){return set_v4l_parameter(param, std::to_string(value));}
    static void adjust_camera();
    static bool start_capture();
    static camera_image_t* read_frame();
    static bool decode_ffmpeg(const void *src, int len, camera_image_t *dest);
    static bool process_image(const void * src, int len, camera_image_t * dest);
    static bool suspend();
    static void release_device();
    static void close_handlers();
public:
    static std::vector<capture_format_t>& get_supported_formats();
    static UsbCam& Instance();
};

}

#endif

