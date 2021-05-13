# usb_cam [![ROS2 CI](https://github.com/ros-drivers/usb_cam/actions/workflows/build_test.yml/badge.svg)](https://github.com/ros-drivers/usb_cam/actions/workflows/build_test.yml)

## A ROS2 Driver for V4L USB Cameras
This package is based off of V4L devices specifically instead of just UVC.

For ros1 documentation, see [the ROS wiki](http://ros.org/wiki/usb_cam).

## Supported ROS2 Distros and Platforms

All Officially supported Linux Distros and corresponding ROS2 releases are supported. Please create an issue if you experience any problems on these platforms.

Windows: TBD/Untested/Unproven
MacOS: TBD/Untested/Unproven

For either MacOS or Windows - if you would like to try and get it working please create an issue to document your effort. If it works we can add it to the instructions here!

## Quickstart

**This package is still not yet released for any ROS2 Distros yet!**

Assuming you have a supported ROS2 distro installed, run the following command to install the binary release:

```bash
sudo apt get install ros-<ros2-distro>-usb-cam
```


## Building from Source

Clone/Download the source code into your workspace:

```
cd /path/to/catkin_ws/src
git clone https://github.com/ros-drivers/usb_cam.git
```

Or click on the green "Download zip" button on the repo's github webpage.

Once downloaded and ensuring you have sourced your ROS2 underlay, go ahead and compile:

```
cd /path/to/colcon_ws
colcon build
source /path/to/colcon_ws/install/setup.bash
```

Be sure to source the newly built packages after a successful build.
Once sourced, you should be able to run the package in one of three ways, shown in the next section

## Running

The `usb_cam_node` can be ran with default settings, by setting specific parameters either via the command line or by loading in a file. We provide a "default" params file in the `usb_cam/config/params.yaml` directory to get you started. Also provided is a launch file that should launch the `usb_cam_node` Node along with a provided node that displays an image topic. The commands to run each of these different ways of starting the node are shown below:

**NOTE: you only need to run ONE of the commands below to run the node**
```
ros2 run usb_cam usb_cam_node # this will run the node with default settings (without params file)
ros2 run usb_cam usb_cam_node --ros-args --params-file /path/to/colcon_ws/src/usb_cam/config/params.yaml   # with params file
ros2 launch usb_cam demo_launch.py  # also runs the provided show_images.py script to display the image topic
```

#### Documentation

[Doxygen](http://docs.ros.org/indigo/api/usb_cam/html/) files can be found on the ROS wiki.

### License
usb_cam is released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.

### Authors
See the [AUTHORS](AUTHORS.md) file for a full list of contributors.
