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

#include <ros/forwards.h>
#include "usb_cam/camera_driver.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

namespace usb_cam
{

class UsbCam: public AbstractV4LUSBCam
{
private:
    /* SINGLETON */
    explicit UsbCam();
    virtual ~UsbCam();
    UsbCam(const UsbCam& root) = delete;
    UsbCam operator=(const UsbCam& root) = delete;
protected:
    static bool create_suspended;

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
    ros::ServiceServer _service_supported_controls;
    static ros::ServiceServer* service_supported_controls;
    static bool service_supported_controls_callback(std_srvs::Trigger::Request& request,
                                                    std_srvs::Trigger::Response& response);

    /* Node parameters */
    static std::string camera_name;
    static std::string camera_frame_id;
    static std::string camera_transport_suffix;
    static std::string camera_info_url;

public:
    static UsbCam& Instance();
};

}

#endif

