#include <cstdlib>

#include <mutex>
#include <sstream>
#include <thread>

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <nodelet/NodeletUnload.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <system_error>

#include <usb_cam/usb_cam.h>

namespace usb_cam
{
static const std::string DEFAULT_CAMERA_NAME = "head_camera";

static const std::string DEFAULT_VIDEO_DEVICE = "/dev/video0";
static const std::string DEFAULT_IO_METHOD = "mmap";
static const std::string DEFAULT_PIXEL_FORMAT = "mjpeg";

static const int DEFAULT_IMAGE_HEIGHT = 480;
static const int DEFAULT_IMAGE_WIDTH = 640;

static const int DEFAULT_FRAMERATE = 30;

static const int DEFAULT_EXPOSURE = 100;

static const int DEFAULT_WHITE_BALANCE = 4000;

static const bool DEFAULT_AUTO_FOCUS_STATE = false;
static const bool DEFAULT_AUTO_EXPOSURE_STATE = true;
static const bool DEFAULT_AUTO_WHITE_BALANCE_STATE = true;

static const int AUTOMATIC = -1;

class UsbCamNodelet : public nodelet::Nodelet
{
public:
    UsbCamNodelet() :
            img_(new sensor_msgs::Image())
    {

    }

    void onInit()
    {
        ros::NodeHandle nh = getPrivateNodeHandle();

        image_transport::ImageTransport it(nh);

        image_pub_ = it.advertiseCamera("image_raw", 1);

        load_from_parameters_(nh);

        cinfo_.reset(new camera_info_manager::CameraInfoManager(nh, camera_name_, camera_info_url_));

        if (!cinfo_->isCalibrated())
        {
            recalibrate_camera_info_();
        }

        NODELET_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS",
                     camera_name_.c_str(), video_device_name_.c_str(), image_width_, image_height_,
                     io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);

        io_method_ = UsbCam::io_method_from_string(io_method_name_);
        if(io_method_ == UsbCam::IO_METHOD_UNKNOWN)
        {
            NODELET_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
            return;
        }

        pixel_format_ = UsbCam::pixel_format_from_string(pixel_format_name_);
        if (pixel_format_ == UsbCam::PIXEL_FORMAT_UNKNOWN)
        {
            NODELET_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
            return;
        }

        service_toggle_capture_ = nh.advertiseService("toggle_capture", &UsbCamNodelet::service_toggle_cap, this);
    }

    virtual ~UsbCamNodelet()
    {
        service_toggle_capture_.shutdown();
        std::unique_lock<std::mutex> cam_lock(cam_mutex_);
        cam_.shutdown();
        cam_lock.unlock();
        if (publisher_thread_.joinable())
        {
            publisher_thread_.join();
        }
    }

    bool service_toggle_cap(std_srvs::SetBool::Request & req, std_srvs::SetBool::Response & res)
    {
        if (req.data)
        {
            std::unique_lock<std::mutex> cam_lock(cam_mutex_);
            if (!cam_.is_capturing())
            {
                // if capturing was recently stopped, we should wait for thread to finish execution
                cam_.start(video_device_name_.c_str(), io_method_, pixel_format_, image_width_,
		               image_height_, framerate_);
                set_v4l_params_();
                cam_lock.unlock();
                try
                {
                    if (publisher_thread_.joinable())
                    {
                        publisher_thread_.join();
                    }
                    cam_lock.lock();
                    cam_.start_capturing();
                    publisher_thread_ = std::thread(&UsbCamNodelet::spin, this);
                    res.success = true;
                }
                catch (const std::system_error & e)
                {
                    NODELET_ERROR("Couldn't start image publisher thread - %s", e.code().message().c_str());
                }
            }
        }
        else
        {
            std::lock_guard<std::mutex> cam_lock(cam_mutex_);
            if (cam_.is_capturing()) {
                cam_.shutdown();
            }
            res.success = true;
        }
        return res.success;
    }

    void spin()
    {
        if(io_method_ == UsbCam::IO_METHOD_UNKNOWN || pixel_format_ == UsbCam::PIXEL_FORMAT_UNKNOWN)
        {
            NODELET_ERROR("Unknown IO method % s or Pixel format %s", io_method_name_, pixel_format_name_);
            return;
        }

        ros::Rate loop_rate(framerate_);
        try
        {
            while (true)
            {
                std::unique_lock<std::mutex> cam_lock(cam_mutex_);
                if (cam_.is_capturing())
                {
                    if (!take_and_send_image_()) {
                        NODELET_WARN("USB camera did not respond in time.");
                    }
                }
                else
                {
                    return;
                }
                cam_lock.unlock();
                loop_rate.sleep();
            }
        }
        catch (const std::exception & e)
        {
            NODELET_ERROR("Exception: %s", e.what());
            return;
        }
    }

private:
    // Not thread-safe, called from spin()
    bool take_and_send_image_()
    {
        cam_.grab_image(img_.get());

        sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
        ci->header.frame_id = img_->header.frame_id;
        ci->header.stamp = img_->header.stamp;

        image_pub_.publish(img_, ci);

        return true;
    }

    void load_from_parameters_(ros::NodeHandle & nh)
    {
        nh.param("video_device", video_device_name_, DEFAULT_VIDEO_DEVICE);
        nh.param("brightness", brightness_, AUTOMATIC); //0-255
        nh.param("contrast", contrast_, AUTOMATIC); //0-255
        nh.param("saturation", saturation_, AUTOMATIC); //0-255
        nh.param("sharpness", sharpness_, AUTOMATIC); //0-255
        // possible values: mmap, read, userptr
        nh.param("io_method", io_method_name_, DEFAULT_IO_METHOD);
        nh.param("image_width", image_width_, DEFAULT_IMAGE_WIDTH);
        nh.param("image_height", image_height_, DEFAULT_IMAGE_HEIGHT);
        nh.param("framerate", framerate_, DEFAULT_FRAMERATE);
        // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
        nh.param("pixel_format", pixel_format_name_, DEFAULT_PIXEL_FORMAT);
        // enable/disable autofocus
        nh.param("autofocus", autofocus_, DEFAULT_AUTO_FOCUS_STATE);
        nh.param("focus", focus_, AUTOMATIC); //0-255
        // enable/disable autoexposure
        nh.param("autoexposure", autoexposure_, DEFAULT_AUTO_EXPOSURE_STATE);
        nh.param("exposure", exposure_, DEFAULT_EXPOSURE);
        nh.param("gain", gain_, AUTOMATIC); //0-100?
        // enable/disable auto white balance temperature
        nh.param("auto_white_balance", auto_white_balance_, true);
        nh.param("white_balance", white_balance_, DEFAULT_WHITE_BALANCE);

        // load the camera info
        nh.param("camera_frame_id", img_->header.frame_id, DEFAULT_CAMERA_NAME);
        nh.param("camera_name", camera_name_, DEFAULT_CAMERA_NAME);
        nh.param("camera_info_url", camera_info_url_, std::string());
    }

    void recalibrate_camera_info_()
    {
        sensor_msgs::CameraInfo calibration;
        calibration.header.frame_id = img_->header.frame_id;
        calibration.width = image_width_;
        calibration.height = image_height_;

        cinfo_->setCameraName(video_device_name_);
        cinfo_->setCameraInfo(calibration);
    }

    // Not thread-safe, call before starting publisher_thread_
    void set_v4l_params_()
    {
        if (brightness_ >= 0)
        {
            cam_.set_v4l_parameter("brightness", brightness_);
        }

        if (contrast_ >= 0)
        {
            cam_.set_v4l_parameter("contrast", contrast_);
        }

        if (saturation_ >= 0)
        {
            cam_.set_v4l_parameter("saturation", saturation_);
        }

        if (sharpness_ >= 0)
        {
            cam_.set_v4l_parameter("sharpness", sharpness_);
        }

        if (gain_ >= 0)
        {
            cam_.set_v4l_parameter("gain", gain_);
        }

        // check auto white balance
        if (auto_white_balance_)
        {
            cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
        }
        else
        {
            cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
            cam_.set_v4l_parameter("white_balance_temperature", white_balance_);
        }

        // check auto exposure
        if (!autoexposure_)
        {
            // turn down exposure control (from max of 3)
            cam_.set_v4l_parameter("exposure_auto", 1);
            // change the exposure level
            cam_.set_v4l_parameter("exposure_absolute", exposure_);
        }

        // check auto focus
        if (autofocus_)
        {
            cam_.set_auto_focus(1);
            cam_.set_v4l_parameter("focus_auto", 1);
        }
        else
        {
            cam_.set_v4l_parameter("focus_auto", 0);
            if (focus_ >= 0)
            {
              cam_.set_v4l_parameter("focus_absolute", focus_);
            }
        }
    }

    sensor_msgs::ImagePtr img_;
    image_transport::CameraPublisher image_pub_;

    std::string video_device_name_;
    std::string io_method_name_;
    std::string pixel_format_name_;
    std::string camera_name_;
    std::string camera_info_url_;

    bool streaming_status_;

    int image_width_;
    int image_height_;
    int framerate_;
    int exposure_;
    int brightness_;
    int contrast_;
    int saturation_;
    int sharpness_;
    int focus_;
    int white_balance_;
    int gain_;

    bool autofocus_;
    bool autoexposure_;
    bool auto_white_balance_;

    UsbCam::io_method io_method_;
    UsbCam::pixel_format pixel_format_;

    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

    UsbCam cam_;
    std::mutex cam_mutex_;

    ros::ServiceServer service_toggle_capture_;

    std::thread publisher_thread_;
};
}

PLUGINLIB_EXPORT_CLASS(usb_cam::UsbCamNodelet, nodelet::Nodelet);