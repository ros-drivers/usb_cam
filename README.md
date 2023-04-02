# usb_cam [![ROS 2 CI](https://github.com/ros-drivers/usb_cam/actions/workflows/build_test.yml/badge.svg)](https://github.com/ros-drivers/usb_cam/actions/workflows/build_test.yml)

## A ROS 2 Driver for V4L USB Cameras
This package is based off of V4L devices specifically instead of just UVC.

For ros1 documentation, see [the ROS wiki](http://ros.org/wiki/usb_cam).

## Supported ROS 2 Distros and Platforms

All Officially supported Linux Distros and corresponding ROS 2 releases are supported. Please create an issue if you experience any problems on these platforms.

Windows: TBD/Untested/Unproven
MacOS: TBD/Untested/Unproven

For either MacOS or Windows - if you would like to try and get it working please create an issue to document your effort. If it works we can add it to the instructions here!

## Quickstart

Assuming you have a supported ROS 2 distro installed, run the following command to install the binary release:

```bash
sudo apt-get install ros-<ros2-distro>-usb-cam
```

As of today this package should be available for binary installation on all active ROS 2 distros.

If for some reason you cannot install the binaries, follow the directions below to compile from source.

## Building from Source

Clone/Download the source code into your workspace:

```
cd /path/to/colcon_ws/src
git clone https://github.com/ros-drivers/usb_cam.git
```

Or click on the green "Download zip" button on the repo's github webpage.

Once downloaded and ensuring you have sourced your ROS 2 underlay, go ahead and install the dependencies:

```
cd /path/to/colcon_ws
rosdep install --from-paths src --ignore-src -y
```

From there you should have all the necessary dependencies installed to compile the `usb_cam` package:

```
cd /path/to/colcon_ws
colcon build
source /path/to/colcon_ws/install/setup.bash
```

Be sure to source the newly built packages after a successful build.

Once sourced, you should be able to run the package in one of three ways, shown in the next section.

## Running

The `usb_cam_node` can be ran with default settings, by setting specific parameters either via the command line or by loading in a parameters file.

We provide a "default" params file in the `usb_cam/config/params.yaml` directory to get you started. Feel free to modify this file as you wish.

Also provided is a launch file that should launch the `usb_cam_node_exe` executable along with an additional node that displays an image topic.

The commands to run each of these different ways of starting the node are shown below:

**NOTE: you only need to run ONE of the commands below to run the node**

```
# run the executable with default settings (without params file)
ros2 run usb_cam usb_cam_node_exe

# run the executable while passing in parameters via a yaml file
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /path/to/colcon_ws/src/usb_cam/config/params.yaml

# launch the usb_cam executable that loads parameters from the same `usb_cam/config/params.yaml` file as above
# along with an additional image viewer node
ros2 launch usb_cam demo_launch.py
```
## Launching Multiple usb_cam's

To launch multiple nodes at once, simply remap the namespace of each one:

```
ros2 run usb_cam usb_cam_node_exe --remap __ns:=/usb_cam_0 --params-file /path/to/usb_cam/config/params_0.yaml
ros2 run usb_cam usb_cam_node_exe --remap __ns:=/usb_cam_1 --params-file /path/to/usb_cam/config/params_1.yaml
```

## Supported formats

### Device supported formats

To see a connected devices supported formats, run the `usb_cam_node` and observe the console output.

An example output is:

```
[INFO] [1680452417.506838265] [usb_cam]: This devices supproted formats:
[INFO] [1680452417.507199183] [usb_cam]:        Motion-JPEG: 1280 x 720 (30 Hz)
[INFO] [1680452417.507225494] [usb_cam]:        Motion-JPEG: 960 x 540 (30 Hz)
[INFO] [1680452417.507239725] [usb_cam]:        Motion-JPEG: 848 x 480 (30 Hz)
[INFO] [1680452417.507252414] [usb_cam]:        Motion-JPEG: 640 x 480 (30 Hz)
[INFO] [1680452417.507269112] [usb_cam]:        Motion-JPEG: 640 x 360 (30 Hz)
[INFO] [1680452417.507281981] [usb_cam]:        YUYV 4:2:2: 640 x 480 (30 Hz)
[INFO] [1680452417.507296745] [usb_cam]:        YUYV 4:2:2: 1280 x 720 (10 Hz)
[INFO] [1680452417.507311292] [usb_cam]:        YUYV 4:2:2: 640 x 360 (30 Hz)
[INFO] [1680452417.507325806] [usb_cam]:        YUYV 4:2:2: 424 x 240 (30 Hz)
[INFO] [1680452417.507339344] [usb_cam]:        YUYV 4:2:2: 320 x 240 (30 Hz)
[INFO] [1680452417.507354164] [usb_cam]:        YUYV 4:2:2: 320 x 180 (30 Hz)
[INFO] [1680452417.507368658] [usb_cam]:        YUYV 4:2:2: 160 x 120 (30 Hz)
```

### Driver supported formats

The driver has its own supported formats. See [the source code](include/usb_cam/formats/)
for details.

After observing [the devices supported formats](#device-supported-formats), specify which
format to use via [the parameters file](config/params.yaml) with the `pixel_format` parameter.

Possible options for this driver today are:

- `yuyv2rgb`: V4L2 capture format of YUYV, ROS image encoding of RGB8
- `uyvy2rgb`: V4L2 capture format of UYVY, ROS image encoding of RGB8
- `mjpeg2rgb`: V4L2 capture format of MJPEG, ROS image encoding of RGB8
- `rgb8`: V4L2 capture format and ROS image encoding format of RGB8
- `yuyv`: V4L2 capture format and ROS image encoding format of YUYV
- `uyvy`: V4L2 capture format and ROS image encoding format of UYVY
- `m4202rgb8`: V4L2 capture format of M420 (aka YUV420), ROS image encoding of RGB8
- `mono8`: V4L2 capture format and ROS image encoding format of MONO8
- `mono16`: V4L2 capture format and ROS image encoding format of MONO16
- `y102mono8`: V4L2 capture format of Y10 (aka MONO10), ROS image encoding of MONO8

More formats and conversions can be added, contributions welcome!

## Compression

Big thanks to [the `ros2_v4l2_camera` package](https://gitlab.com/boldhearts/ros2_v4l2_camera#usage-1) and their documentation on this topic.

The `usb_cam` should support compression by default since it uses `image_transport` to publish its images as long as the `image_transport_plugins` package is installed on your system. With the plugins installed the `usb_cam` package should publish a `compressed` topic automatically.

Unfortunately `rviz2` and `show_image.py` do not support visualizing the compressed images just yet so you will need to republish the compressed image downstream to uncompress it:

```
ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=image_raw/compressed --remap out:=image_raw/uncompressed
```

#### Documentation

[Doxygen](http://docs.ros.org/indigo/api/usb_cam/html/) files can be found on the ROS wiki.

### License
usb_cam is released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.

### Authors
See the [AUTHORS](AUTHORS.md) file for a full list of contributors.
