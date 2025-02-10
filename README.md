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

```shell
sudo apt-get install ros-<ros2-distro>-usb-cam
```

As of today this package should be available for binary installation on all active ROS 2 distros.

If for some reason you cannot install the binaries, follow the directions below to compile from source.

## Building from Source

Clone/Download the source code into your workspace:

```shell
cd /path/to/colcon_ws/src
git clone https://github.com/ros-drivers/usb_cam.git
```

Or click on the green "Download zip" button on the repo's github webpage.

Once downloaded and ensuring you have sourced your ROS 2 underlay, go ahead and install the dependencies:

```shell
cd /path/to/colcon_ws
rosdep install --from-paths src --ignore-src -y
```

From there you should have all the necessary dependencies installed to compile the `usb_cam` package:

```shell
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

```shell
# run the executable with default settings (without params file)
ros2 run usb_cam usb_cam_node_exe

# run the executable while passing in parameters via a yaml file
ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /path/to/colcon_ws/src/usb_cam/config/params.yaml

# launch the usb_cam executable that loads parameters from the same `usb_cam/config/params.yaml` file as above
# along with an additional image viewer node
ros2 launch usb_cam camera.launch.py
```
## Launching Multiple usb_cam's

To launch multiple nodes at once, simply remap the namespace of each one:

```shell
ros2 run usb_cam usb_cam_node_exe --ros-args --remap __ns:=/usb_cam_0 --params-file /path/to/usb_cam/config/params_0.yaml
ros2 run usb_cam usb_cam_node_exe --ros-args --remap __ns:=/usb_cam_1 --params-file /path/to/usb_cam/config/params_1.yaml
```

## Supported formats

### Device supported formats

To see a connected devices supported formats, run the `usb_cam_node` and observe the console output.

An example output is:

```log
This devices supproted formats:
       Motion-JPEG: 1280 x 720 (30 Hz)
       Motion-JPEG: 960 x 540 (30 Hz)
       Motion-JPEG: 848 x 480 (30 Hz)
       Motion-JPEG: 640 x 480 (30 Hz)
       Motion-JPEG: 640 x 360 (30 Hz)
       YUYV 4:2:2: 640 x 480 (30 Hz)
       YUYV 4:2:2: 1280 x 720 (10 Hz)
       YUYV 4:2:2: 640 x 360 (30 Hz)
       YUYV 4:2:2: 424 x 240 (30 Hz)
       YUYV 4:2:2: 320 x 240 (30 Hz)
       YUYV 4:2:2: 320 x 180 (30 Hz)
       YUYV 4:2:2: 160 x 120 (30 Hz)
```

### Driver supported formats

The driver has its own supported formats. See [the source code](include/usb_cam/formats/)
for details.

After observing [the devices supported formats](#device-supported-formats), specify which
format to use via [the parameters file](config/params.yaml) with the `pixel_format` parameter.

To see a list of all currently supported driver formats, run the following command:

```shell
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:="test"
```

Note: "test" here could be replaced with any non-supported pixel format string. The driver
will detect if the given pixel format is supported or not.

More formats and conversions can be added, contributions welcome!

### Supported IO methods

This driver supports three different IO methods as of today:

1. `read`: copies the video frame between user and kernal space
1. `mmap`: memory mapped buffers allocated in kernel space
1. `userptr`: memory buffers allocated in the user space

To read more on the different methods, check out [this article that provides a good overview
of each](https://lwn.net/Articles/240667/)

## Compression

Big thanks to [the `ros2_v4l2_camera` package](https://gitlab.com/boldhearts/ros2_v4l2_camera#usage-1) and their documentation on this topic.

The `usb_cam` should support compression by default since it uses `image_transport` to publish its images as long as the `image_transport_plugins` package is installed on your system. With the plugins installed the `usb_cam` package should publish a `compressed` topic automatically.

Unfortunately `rviz2` and `show_image.py` do not support visualizing the compressed images just yet so you will need to republish the compressed image downstream to uncompress it:

```shell
ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=image_raw/compressed --remap out:=image_raw/uncompressed
```

## Testing

To run the basic unit tests for this repository:

```shell
colcon build --packages-select usb_cam
colcon test --pacakges-select usb_cam
```

### Integration tests

To run integration tests for this repository:

```shell
colcon build --packages-select usb_cam --cmake-args -DINTEGRATION_TESTS=1
colcon test --pacakges-select usb_cam
```

### Address and leak sanitizing

Incorporated into the `CMakelists.txt` file to assist with memory leak and address sanitizing
is a flag to add these compile commands to the targets.

To enable them, pass in the `SANITIZE=1` flag:

```shell
colcon build --packages-select usb_cam --cmake-args -DSANITIZE=1
```

Once built, run the nodes executable directly and pass any `ASAN_OPTIONS` that are needed:

```shell
ASAN_OPTIONS=new_delete_type_mismatch=0 ./install/usb_cam/lib/usb_cam/usb_cam_node_exe 
```

After shutting down the executable with `Ctrl+C`, the sanitizer will report any memory leaks.

By default this is turned off since compiling with the sanatizer turned on causes bloat and slows
down performance.

## Documentation

[Doxygen](http://docs.ros.org/indigo/api/usb_cam/html/) files can be found on the ROS wiki.

### License

usb_cam is released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.

### Authors

See the [AUTHORS](AUTHORS.md) file for a full list of contributors.
