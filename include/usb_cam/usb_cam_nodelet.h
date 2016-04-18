#ifndef __USBCAM_NODELET_H
#define __USBCAM_NODELET_H

#include <nodelet/nodelet.h>

#include <usb_cam/usb_cam.h>

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>

namespace usb_cam {
  class UsbCamNodelet : public nodelet::Nodelet
  {
  public:
    UsbCamNodelet();
    ~UsbCamNodelet();

    virtual void onInit();

  private:
    ros::NodeHandle node_;
  
    // shared image message
    sensor_msgs::Image img_;
    image_transport::CameraPublisher image_pub_;
  
    // parameters
    std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_;
    int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
        white_balance_, gain_;
    bool autofocus_, autoexposure_, auto_white_balance_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
  
    UsbCam cam_;

  private:
    bool take_and_send_image();
    bool spin();

    boost::scoped_ptr<boost::thread> stream_thread_;
  };
}

#endif // infdef __USBCAM_NODELET_H

