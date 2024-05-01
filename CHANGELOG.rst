^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package usb_cam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.1 (2024-05-01)
------------------
* Merge pull request `#330 <https://github.com/ros-drivers/usb_cam/issues/330>`_ from ros-drivers/fix-rolling-builds
  Update comment in CI from Jammy to Noble
* Fix linter error in uyvy file
  - Update comment in CI from Jammy to Noble
* Merge pull request `#324 <https://github.com/ros-drivers/usb_cam/issues/324>`_ from clalancette/clalancette/const-avcodec
  Switch to a const AVCodec *.
* Switch to a const AVCodec *.
  This is because newer versions of avcodec return a const AVCodec *.
* Merge pull request `#313 <https://github.com/ros-drivers/usb_cam/issues/313>`_ from ros-drivers/v4l2-devices-might-not-be-named-video
  V4l2 devices might not be named video
* Use /sys/class/video4linux/ to get list of v4l2 devices
* Merge pull request `#311 <https://github.com/ros-drivers/usb_cam/issues/311>`_ from firesurfer/ros2
  Resolve Symlinks
* Update usb_cam_node.cpp
  Fix formatting
* try to fix formatting issue
* resolve symlinks
* Merge pull request `#305 <https://github.com/ros-drivers/usb_cam/issues/305>`_ from ros-drivers/fix-docs-deployment
* Remove unnecessary steps from docs CI
* Merge pull request `#304 <https://github.com/ros-drivers/usb_cam/issues/304>`_ from ros-drivers/fix-docs-ci
  Fix docs ci
* Trigger docs CI on every push to ros2 branch
* Merge pull request `#303 <https://github.com/ros-drivers/usb_cam/issues/303>`_ from ros-drivers/add-mkdocs-documentation
* Standup basic mkdocs documentation site
* Contributors: Chris Lalancette, Evan Flynn, Lennart Nachtigall

0.8.0 (2023-12-04)
------------------
* Bump for release 0.8.0, update docs
* Merge pull request `#300 <https://github.com/ros-drivers/usb_cam/issues/300>`_ from ros-drivers/263-check-if-specified-pixel-format-is-supported
  263 check if specified pixel format is supported
* Check if specified pixel format is supported by driver and by device
  - Also fix typo in params_2 yaml file
  - fix linter errors too
* Add v4l2_str method to pixel_format_base class
* Add common arguments strut to pixel format classes
  - Use common arguments struct in all format classes
* Merge pull request `#292 <https://github.com/ros-drivers/usb_cam/issues/292>`_ from ros-drivers/288-fix-uyvy2rgb-size
  288 fix uyvy2rgb size
* Merge pull request `#270 <https://github.com/ros-drivers/usb_cam/issues/270>`_ from boitumeloruf/raw-mjpeg-stream
  Publish raw mjpeg stream directly via compressed image topic
* Introduced funtions get\_..._from_av_format, fixed code style errors
* Merge branch 'ros2' into raw-mjpeg-stream
* fixed minor build issues
* Merge branch 'ros2' into raw-mjpeg-stream
* Merge branch 'ros2' of https://github.com/boitumeloruf/usb_cam into ros2
* Merge pull request `#295 <https://github.com/ros-drivers/usb_cam/issues/295>`_ from ros-drivers/run-ci-on-ros2-branch-too
  Run CI on ros2 branch pushes too
* Bump checkout action to latest
* Ensure ROS_VERSION is set for ROS build farms
* Run CI on ros2 branch pushes too
* Merge pull request `#294 <https://github.com/ros-drivers/usb_cam/issues/294>`_ from ros-drivers/291-auto-generate-ci-matrix
  Auto generate ci matrix
* Continue on error for ROS 1 build and test job
* Hard-code noetic to actions matrix step until 2025
* Handle rolling case where two docker images are listed
* Switch to tagged version of the active_ros_distros script
* Add basic ROS 1 node, update CMakelists and package.xml
* Update CI to automatically generate distro matrix
* Merge pull request `#293 <https://github.com/ros-drivers/usb_cam/issues/293>`_ from ros-drivers/add-mjpeg-device-format-param
  Add mjpeg device format param
* fixed cppcheck and uncrustify errors
* Refactored use of av_device_format
* Updated example YAML files to av_device_format
* Added av_device_format parameter
* Fix bytes per line logic for base image class
* Fix number of channels for uyvy2rgb format
* Merge pull request `#286 <https://github.com/ros-drivers/usb_cam/issues/286>`_ from ros-drivers/285-handle-unavailable-device
  285 handle unavailable device, list available v4l2 devices
* Fix manual triggering of CI pipelines
* Check if given v4l2 device exists before opening it
* fixed cppcheck and uncrustify errors
* Merge pull request `#282 <https://github.com/ros-drivers/usb_cam/issues/282>`_ from flynneva/ros2-prepare-release
  Bump to 0.7.0, generate CHANGELOG
* Merge branch 'ros2' into raw-mjpeg-stream
* Fix seg fault when unref av_packet using av_codec < 58.133.100
* Merge branch 'ros2' into raw-mjpeg-stream
* Refactored use of av_device_format
* Merge branch 'ros-drivers:ros2' into ros2
* Added feature to access raw mjpeg stream and publish directly on compressed image topic
* Updated example YAML files to av_device_format
* Added av_device_format parameter
* Contributors: Boitumelo Ruf, Evan Flynn

0.7.0 (2023-08-30)
------------------
* Fix mjpeg invalid ptr and mjpeg memory leak
* Allocate unique avpacket for each frame
* Fix some minor memorly leaks for mjpeg
  Relates to `#262 <https://github.com/ros-drivers/usb_cam/issues/262>`_
* Update docs to new launch file name
  Closes `#277 <https://github.com/ros-drivers/usb_cam/issues/277>`_
* Only unref packet in destructor
  Closes `#274 <https://github.com/ros-drivers/usb_cam/issues/274>`_ `#275 <https://github.com/ros-drivers/usb_cam/issues/275>`_
* Enable manaul trigger of ROS 2 CI, add Iron, deprecate Foxy
* Add Iron to CI, remove Foxy
* Enable manaul trigger of ROS 2 CI
* Fix memory leaks in mjpeg2rgb conversion
* Add SANITIZE option to package to help with debugging, document it
* Fix memory leaks caused by buffer allocation by using smart pointers
* Fix linter errors
* Update params2 file for second camera
* Fixed wrong image timestamp due to missing handling of microseconds in epoch time shift
* Removed debug output of timestamp
* Fixed wring image timestamp due to missing handling of microseconds in epoch time shift.
* Address multiple memory leak issues after ros2 rewrite
* Remove EOL Galactic distro from CI
* Address multiple memory leak issues after ros2 rewrite
* Create CameraConfig class, use it in launch file
* imports no longer needed.
* Multiple cameras + compression
* Remove debug print accidentally added
* Clean up ROS 2 node, update parameter logic
* Contributors: Boitumelo Ruf, Brendon Cintas, Evan Flynn

0.6.0 (2023-04-02)
------------------
* If auto exposure is true, set it
* Migrate previous pixel formats to new approach
  - Add M4202RGB pixel format (aka YUV420 to RGB8)
  - Add Y102MONO8 pixel format (aka MONO10 to MONO8)
* Update documentation related to supported formats
  - update doc strings in new pixel format base class
* Fix linter errors, clean up tests
  - fix humble and rolling build
* Implement new pixel_format class structure
  - implement virtual convert method for new pixel format class
  - fix MJPEG2RGB conversion logic using new pixel format class
* Fix typo in workspace path in README
* fix whitespace around comments
* fix unused variable error
* possible fix for timestamp jumping
* fix code style
* dont change brightness with default config
* use v4l2  for "brightness", "contrast", "saturation", "sharpness", "gain", "auto_white_balance",
  "white_balance", "autoexposure", "exposure", "autofocus", "focus"
* Contributors: Evan Flynn, john

0.5.0 (2023-01-14)
------------------
* Merge pull request `#212 <https://github.com/flynneva/usb_cam/issues/212>`_ from flynneva/203-refactor-usb-cam-library-with-no-ros-deps
  Improve ros2 rewrite some more
* Ensure usb_cam lib and node are installed
* Add missing include to test_usb_cam_lib
* Rename format enums to make code easier to read
* Make supported formats a member variable of the UsbCam class
* Adjust get_image method to return image to reduce required args
* Use unused arguments in MJPEG2RGB conversion function
* Treat all compiler warnings as errors to be more strict
* Improve logging errors for usb_cam lib
* Fix opencv include path
* Merge pull request `#210 <https://github.com/flynneva/usb_cam/issues/210>`_ from revanthsenthil/ros2
  ROS 2 installation instruction typo fix
* Merge branch 'ros-drivers:ros2' into ros2
* Merge pull request `#209 <https://github.com/flynneva/usb_cam/issues/209>`_ from flynneva/203-refactor-usb-cam-library-with-no-ros-deps
  Refactor usb cam library with no ros deps
* Update README.md
  resolved error with `apt-get` from `apt get`
* Add back in missing copyrights
* Improve supported formats method for UsbCam object
* Fix MJPEG2RGB conversion function
* Enable code coverage using lcov
* Add integration test for usb_cam lib
* Clean up usb_cam lib, remove rclcpp dep
* Bump default framerate to 30hz
* Improve CLIPVALUE method, add unit test for it
* Fix humble CI name
* Add some basic unit tests to usb_cam
* Remove ROS dep from usb_cam by rewriting timestamping of frames
* Restructure usb_cam code into more digestible pieces
* Merge pull request `#207 <https://github.com/flynneva/usb_cam/issues/207>`_ from flynneva/fix-compiler-warnings
  Fix compiler warnings, replace deprecated code
* Add basic linters to CMake, fix linter errors found
* Fix compiler warnings, replace deprecated code
* Merge pull request `#206 <https://github.com/flynneva/usb_cam/issues/206>`_ from flynneva/ros2
  Add humble to CI
* Add humble to CI
* Merge pull request `#177 <https://github.com/flynneva/usb_cam/issues/177>`_ from benmaidel/feature/YUV420_ros2
  [ros2] add support for YUV420 (yu12) pixel format
* Merge pull request `#193 <https://github.com/flynneva/usb_cam/issues/193>`_ from mad0x60/patch-1
* update the deprecated uncompressed command
  The current ros2 image decompression command produces the following warning because it is deprecated:
  [WARN] [1662133933.155713605] [rcl]: Found remap rule 'in/compressed:=image_raw/compressed'. This syntax is deprecated. Use '--ros-args --remap in/compressed:=image_raw/compressed' instead.
  [WARN] [1662133933.155877454] [rcl]: Found remap rule 'out:=image_raw/uncompressed'. This syntax is deprecated. Use '--ros-args --remap out:=image_raw/uncompressed' instead.
  This change update to match the most recent ROS2 format
* Merge pull request `#189 <https://github.com/flynneva/usb_cam/issues/189>`_ from flynneva/ros2
  Suppress libav deprecated pixel format used warnings (backport from #…
* Suppress libav deprecated pixel format used warnings (backport from `#115 <https://github.com/flynneva/usb_cam/issues/115>`_)
* Merge branch 'ros2' into feature/YUV420_ros2
* Merge pull request `#188 <https://github.com/flynneva/usb_cam/issues/188>`_ from krsche/feat/ros2-add-h264-support
* feat: add color_format param to support yuv422p
  required for using cams like the Logitech C920 with the h264 pixel_format
* feat: add h264 support
* Merge pull request `#185 <https://github.com/flynneva/usb_cam/issues/185>`_ from progtologist/ros2
  Enabled dynamic reconfiguration of usb_cam_node
* Added debug print in callback
  Co-authored-by: Evan Flynn <6157095+flynneva@users.noreply.github.com>
* Merge pull request `#186 <https://github.com/flynneva/usb_cam/issues/186>`_ from ros-drivers/prep-for-release
  Prep for release
* Enabled dynamic reconfiguration of usb_cam_node
* Merge branch 'ros2' into feature/YUV420_ros2
* add support for YUV420 (yu12) pixel format
* Contributors: Andrei Vukolov, Aris Synodinos, Benjamin Maidel, Evan Flynn, Fabian Kirschner, Mohamed Moustafa, Revanth S

0.4.2 (2022-04-25)
------------------
* Minor bump for release
* Merge pull request `#184 <https://github.com/ros-drivers/usb_cam/issues/184>`_ from clalancette/clalancette/cleanups
* Switch the rolling docker image to use jammy.
* Add default cases to switches.
  This just quiets the compiler warnings.
* Switch xioctl to take an unsigned long request.
  This matches what ioctl actually takes, and gets rid of a sign
  comparison warning.
* Use uint32_t to store image sizes.
  This matches the v4l2 structures, and ensures we don't get
  sign warnings when compiling.
* Merge pull request `#178 <https://github.com/ros-drivers/usb_cam/issues/178>`_ from benmaidel/feature/unsupported_set_format_options_ros2
  [ros2] allow cameras that do not support setting format options via VIDIOC_S_FMT
* allow cameras that do not support setting format options via VIDIOC_S_FMT
* Merge pull request `#170 <https://github.com/ros-drivers/usb_cam/issues/170>`_ from kenji-miyake/fix-small-warnings
  Fix small warnings
* Fix -Wreturn-type
* Fix -Wparentheses
* Change static functions to inline to fix -Wunused-function
* Fix -Wunused-parameter
* Fix -Worder
* Fix -Wcomment
* Fix -Wformat
* add instructions for multiple launches
* Contributors: Benjamin Maidel, Chris Lalancette, Evan Flynn, Kenji Miyake

0.4.1 (2021-08-08)
------------------
* update README, add compression section
* update package.xml to include image_transport_plugins
* clean up README instructions
* update README ros2 run executable name
* Contributors: Evan Flynn

0.4.0 (2021-07-21)
------------------
* bump version for ros2 first release
* add galactic to ci, closes `#157 <https://github.com/ros-drivers/usb_cam/issues/157>`_
  update camera name
* Merge pull request `#158 <https://github.com/ros-drivers/usb_cam/issues/158>`_ from wep21/feature/add_camera_info
  Feature/add camera info
* Add sample camera info yaml
* Add camera info
* Merge pull request `#156 <https://github.com/ros-drivers/usb_cam/issues/156>`_ from wep21/feature/composable_node
* Make usb_cam_node composable
* Merge pull request `#153 <https://github.com/ros-drivers/usb_cam/issues/153>`_ from flynneva/lint/make-utils-file
* add utils file, fix lint errors
* Merge pull request `#151 <https://github.com/ros-drivers/usb_cam/issues/151>`_ from flynneva/fix/remove-boost
* replace boost lexical_cast with snprintf
* Merge pull request `#149 <https://github.com/ros-drivers/usb_cam/issues/149>`_ from flynneva/fix/update-readme
  fix readme headers
* fix readme headers
* Merge pull request `#148 <https://github.com/ros-drivers/usb_cam/issues/148>`_ from flynneva/update-ros2-readme-and-lint
  Update ros2 readme and lint
* fix most lint errors
* update readme, fix linter errors
* Merge pull request `#146 <https://github.com/ros-drivers/usb_cam/issues/146>`_ from flynneva/ros2-clean-up
  Ros2 clean up
* fix show_image script lag
* run, launch and params file working
* add service, install launch, separate header
* Merge pull request `#139 <https://github.com/ros-drivers/usb_cam/issues/139>`_ from flynneva/cmake-cleanup
  consolidate srcs, use ament_auto macros, closes `#138 <https://github.com/ros-drivers/usb_cam/issues/138>`_
* consolidate srcs, use ament_auto macros, closes `#137 <https://github.com/ros-drivers/usb_cam/issues/137>`_ `#138 <https://github.com/ros-drivers/usb_cam/issues/138>`_
* Merge pull request `#132 <https://github.com/ros-drivers/usb_cam/issues/132>`_ from flynneva/foxy
  updates for foxy
* add myself to authors
* fixing lint errors
* add ros2 github actions
* minor updates to foxy
* adding launch file
  try to fix low framerate `#103 <https://github.com/ros-drivers/usb_cam/issues/103>`_
  add ros parameters
  loading more parameters from parameter server `#103 <https://github.com/ros-drivers/usb_cam/issues/103>`_
  use argparse to get arguments from command line
  untested correction to args
  ignore unknown args
  set proper default device and look for more bad return values
  trying to find why framerate is limited to about 8 fps
  framerate ok for low-exposure settings
  print list of valid formats `#105 <https://github.com/ros-drivers/usb_cam/issues/105>`_
* use the v4l2_buffer timestamp if available. `#75 <https://github.com/ros-drivers/usb_cam/issues/75>`_
  usb_cam.cpp is building but untested `#103 <https://github.com/ros-drivers/usb_cam/issues/103>`_
  Builds but crashes immediately after running
  data is getting published, no image shown
  image shown, frame rate is very slow `#103 <https://github.com/ros-drivers/usb_cam/issues/103>`_
* move the timestamping closer to when the image was acquired. `#75 <https://github.com/ros-drivers/usb_cam/issues/75>`_
* Merge pull request `#136 <https://github.com/ros-drivers/usb_cam/issues/136>`_ from flynneva/patch-1
  add myself as a maintainer for ros2
* add myself as a maintainer for ros2
* Merge pull request `#124 <https://github.com/ros-drivers/usb_cam/issues/124>`_ from k-okada/add_noetic
  add noetic .travis.yml
* add noetic .travis.yml
* Contributors: Evan Flynn, Kei Okada, Lucas Walter, flynneva, wep21

0.3.7 (2018-10-29)
------------------
* ROS2 version

0.3.6 (2017-06-15)
------------------
* .travis.yml: udpate to trusty
* add AV\_ prefix to PIX_FMT\_* for X,Y (`#71 <https://github.com/ros-drivers/usb_cam/issues/71>`_)
* Contributors: Kei Okada

0.3.5 (2017-06-14)
------------------
* add ROS Orphaned Package Maintainers to maintainer tag (`#69 <https://github.com/ros-drivers/usb_cam/issues/69>`_)
* support for Kinetic / Ubuntu 16.04 (`#58 <https://github.com/ros-drivers/usb_cam/issues/58>`_)
  * replace use of deprecated functions in newer ffmpeg/libav versions
    ffmpeg/libav 55.x (used in ROS Kinetic) deprecated the avcodec_alloc_frame.
* Add grey scale pixel format. (`#45 <https://github.com/ros-drivers/usb_cam/issues/45>`_)
* add start/stop capture services (`#44 <https://github.com/ros-drivers/usb_cam/issues/44>`_ )
  * better management of start/stop
  * up package.xml
  * add capture service

* fix bug for byte count in a pixel (3 bytes not 24 bytes) (`#40 <https://github.com/ros-drivers/usb_cam/issues/40>`_ )
* Contributors: Daniel Seifert, Eric Zavesky, Kei Okada, Ludovico Russo, Russell Toris, honeytrap15

0.3.4 (2015-08-18)
------------------
* Installs launch files
* Merge pull request #37 from tzutalin/develop
  Add a launch file for easy test
* Add a launch file for easy test
* Contributors: Russell Toris, tzu.ta.lin

0.3.3 (2015-05-14)
------------------
* Merge pull request #36 from jsarrett/develop
  add gain parameter
* add gain parameter
* Contributors: James Sarrett, Russell Toris

0.3.2 (2015-03-24)
------------------
* Merge pull request #34 from eliasm/develop
  fixed check whether calibration file exists
* fixed check whether calibration file exists
* Contributors: Elias Mueggler, Russell Toris

0.3.1 (2015-02-20)
------------------
* Merge pull request #32 from kmhallen/mono8
  Publish YUVMONO10 images as mono8 instead of rgb8
* Publish YUVMONO10 images as mono8 instead of rgb8
* Contributors: Kevin Hallenbeck, Russell Toris

0.3.0 (2015-01-26)
------------------
* Merge pull request #30 from mitchellwills/develop
  Removed global state from usb_cam by encapsulating it inside an object
* Made device name a std::string instead of const char*
* Added usb_cam namespace
* Added underscore sufix to class fields
* Removed camera_ prefix from methods
* Moved methods to parse pixel_format and io_method from string to UsbCam
* Moved camera_image_t struct to be private in UsbCam
* Cleaned up parameter assignment
* Made set_v4l_parameters a non-static function
* Moved set_v4l_parameters to UsbCam object
* Removed global state from usb_cam by encapsulating it inside an object
  function and structions in usb_cam.h became public and everything else is private
* Merge pull request #28 from mitchellwills/develop
  Fix installation of header files
* Fix installation of header files
* Contributors: Mitchell Wills, Russell Toris

0.2.0 (2015-01-16)
------------------
* Bug fix in camera info settings.
* Update .travis.yml
* Merge pull request #27 from bosch-ros-pkg/default_camera_info
  sets default camera info
* sets default camera info
* Contributors: Russell Toris

0.1.13 (2014-12-02)
-------------------
* Merge pull request #25 from blutack/patch-1
  Warn rather than error if framerate can't be set
* Warn rather than error if framerate can't be set
  The driver doesn't currently work with em28xx based devices as they don't allow the framerate to be set directly and the node exits with an error. Changing to a warning allows these devices to be used.
* Update README.md
* Merge pull request #24 from rjw57/do-not-touch-parameters-unless-asked
  do not modify parameters unless explicitly set
* do not modify parameters unless explicitly set
  The contrast, saturation, brightness, sharpness and focus parameters
  were recently added to usb_cam. This caused a regression
  (sigproc/robotic_surgery#17) whereby the default settings for a webcam
  are overridden in all cases by the hard-coded defaults in usb_cam.
  In the absence of a know good set of "default" values, leave the
  parameters unset unless the user has explicitly set them in the launch
  file.
* Contributors: Rich Wareham, Russell Toris, blutack

0.1.12 (2014-11-05)
-------------------
* Merge pull request #22 from dekent/develop
  White balance parameters
* Parameter to enable/disable auto white balance
* Added parameters for white balance
* uses version major to check for av_codec
* uses version header to check for AV_CODEC_ID_MJPEG
* Contributors: David Kent, Russell Toris

0.1.11 (2014-10-30)
-------------------
* Merge pull request #20 from dekent/develop
  More Parameters
* bug fix
* Setting focus when autofocus is disabled
* Parameter adjusting
* Added parameter setting for absolute focus, brightness, contrast, saturation, and sharpness
* Contributors: David Kent, Russell Toris

0.1.10 (2014-10-24)
-------------------
* Merge pull request #19 from bosch-ros-pkg/av_codec_id
  Removed deprecated CODEC_ID
* added legacy macro constants for libav 10
* Renamed deprecated CODEC_ID constants to AV_CODEC_ID to fix compilation for libav 10
* Contributors: Andrzej Pronobis, Russell Toris

0.1.9 (2014-08-26)
------------------
* Uses ros::Rate to enforce software framerate instead of custom time check
* Merge pull request #16 from liangfok/feature/app_level_framerate_control
  Modified to enforce framerate control at the application level in additi...
* Modified to enforce framerate control at the application level in addition to at the driver level.  This is necessary since the drivers for my webcam did not obey the requested framerate.
* Contributors: Russell Toris, liang

0.1.8 (2014-08-21)
------------------
* autoexposure and exposure settings now exposed via ROS parameters
* added ability to call v4l-utils as well as correctly set autofocus
* cleanup of output
* Merge pull request #15 from mistoll/develop
  added support for RGB24 pixel format
* Added RGB24 as pixel format
* Contributors: Michael Stoll, Russell Toris

0.1.7 (2014-08-20)
------------------
* changelog fixed
* minor cleanup and ability to change camera name and info
* Contributors: Russell Toris

0.1.6 (2014-08-15)
------------------
* Merge pull request #14 from KaijenHsiao/master
  added support for 10-bit mono cameras advertising as YUV
* added support for 10-bit mono cameras advertising as YUV (such as Leopard Imaging's LI-USB30-V034)
* Update CHANGELOG.rst
* changelog updated
* Merge pull request #13 from vrabaud/develop
  add a a ros::spinOnce to get set_camera_info working
* add a a ros::spinOnce to get set_camera_info working
  This is explained in the docs of CameraInfoManager
  https://github.com/ros-perception/image_common/blob/hydro-devel/camera_info_manager/include/camera_info_manager/camera_info_manager.h#L71
  Also, this fixes https://github.com/ros-perception/image_pipeline/issues/78
* Contributors: Kaijen Hsiao, Russell Toris, Vincent Rabaud, sosentos

0.1.5 (2014-07-28)
------------------
* auto format
* cleanup of readme and such
* Merge branch 'hydro-devel' of github.com:bosch-ros-pkg/usb_cam
* Merge pull request #11 from pronobis/hydro-devel
  Fixed a bug with av_free missing by adding a proper include.
* Fixed a bug with av_free missing by adding a proper include on Ubuntu 14.04.
* Merge pull request #7 from cottsay/groovy-devel
  Use pkg-config to find avcodec and swscale
* Merge pull request #5 from FriedCircuits/hydro-devel
  Remove requirments for self_test
* Use pkg-config to find avcodec and swscale
* Update package.xml
* Remove selftest
* Remove selftest
* Update usb_cam_node.cpp
* Merge pull request #2 from jonbinney/7_17
  swap out deprecated libavcodec functions
* swap out deprecated libavcodec functions
* Contributors: Andrzej Pronobis, Jon Binney, Russell Toris, Scott K Logan, William

0.1.3 (2013-07-11)
------------------
* Merge pull request #1 from jonbinney/rosify
  Bag of improvements
* add framerate parameter
* use ROS_* for output
* use camera_info_manager
* Contributors: Jon Binney, Russell Toris

0.1.2 (2013-05-06)
------------------
* installs usb_cam_node
* Contributors: Russell Toris

0.1.1 (2013-05-02)
------------------
* cmake fixed
* ffmpeg added
* Contributors: Russell Toris

0.1.0 (2013-05-01)
------------------
* Update package.xml
* minor cleanup
* inital merge
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update README.md
* Update CLONE_SETUP.sh
* Update README.md
* Updated the README.md.
* Updated the installation instructions.
* Fixed syntax in the README.
* Updated README for ARDUINO support.
* Fixed update script.
* Updated the readme and updating scripts.
* Updating for installation on Robot.
* Updated installs and README for ROS.
* Make sure the User knows to source the devel/setup.sh.
* Getting rid of subtrees and Catkinized USB CAM.
* Updating home to use ROSWS.
* Fixing the launch file for video1.
* Merge commit '0bc3322966e4c0ed259320827dd1f5cc8460efce'
  Conflicts:
  src/sofie_ros/package.xml
* Removed unnecessary file.
* Compiles.
* Adding the Catkin build scripts.
* Merge commit 'b2c739cb476e1e01425947e46dc2431464f241b3' as 'src/ar_track_alvar'
* Squashed 'src/ar_track_alvar/' content from commit 9ecca95
  git-subtree-dir: src/ar_track_alvar
  git-subtree-split: 9ecca9558edc7d3a9e692eacc93e082bf1e9a3e6
* Merge commit '9feb470d0ebdaa51e426be4d58f419b45928a671' as 'src/sofie_ros'
* Squashed 'src/sofie_ros/' content from commit 3ca5edf
  git-subtree-dir: src/sofie_ros
  git-subtree-split: 3ca5edfba496840b41bfe01dfdff883cacff1a97
* Removing stackts.
* Removing submodules.
* Fixed submodules.
* Removing old package.
* Merge branch 'catkin'
  Conflicts:
  README.md
  cmake_install.cmake
* Brancing package down to stack base.
* Catkininizing.
* (catkin)Catkininizing.
* Modifying the setup of roshome.
* Starting to Catkininize the project.
* (catkin)Starting to Catkininize the project.
* Going to catinize it.
* (catkin)Going to catinize it.
* Modified to new version of sofie_ros.
* Renamed import_csv_data.py to fileUtils.py, because it does more now.
* (catkin)Renamed import_csv_data.py to fileUtils.py, because it does more now.
* Updating to use a csv file specified by the user. Separating PyTables path manipulation into SOFIEHDFFORMAT.
* (catkin)Updating to use a csv file specified by the user. Separating PyTables path manipulation into SOFIEHDFFORMAT.
* Merge branch 'release/0.0.2'
* Created the install script.
* Removed the Python Packages as submodules.
* Merge branch 'release/0.0.1'
* Update the Git submodules.
* Modified the README and CLONE_SETUP.sh
* Added SOFIEHDFFORMAT as a submodule.
* Added the ExperimentControl Repo as a submodule.
* Working the CLONE install.
* Modifiying install script.
* Added a script to update the gitmodules for read-only clones.
* Merge branch 'master' of github.com:agcooke/roshome
* Initial commit
* Added the modules.
* Added usb_cam,
* Updating to Groovy.
* (catkin)Updating to Groovy.
* Added another potential launch file for exporting video from rosbag.
* (catkin)Added another potential launch file for exporting video from rosbag.
* Added a launcher to ros bag the usb_cam, for later playback.
* (catkin)Added a launcher to ros bag the usb_cam, for later playback.
* Added some files that were possibly not correct
* (catkin)Added some files that were possibly not correct
* Fixed bugs with the importing.
* (catkin)Fixed bugs with the importing.
* Added forgotten __init__.py file and changed to importdata sofiehdfformat funciton.
* (catkin)Added forgotten __init__.py file and changed to importdata sofiehdfformat funciton.
* Refractoring to make it possible to log to CSV.
  There were problems handling concurrent writing to
  pytables files. The package now logs to CSV and then
  provides a function to post import the data into
  SOFIEHDFFORMAT.
* (catkin)Refractoring to make it possible to log to CSV.
  There were problems handling concurrent writing to
  pytables files. The package now logs to CSV and then
  provides a function to post import the data into
  SOFIEHDFFORMAT.
* Exporting to a CSV. Does not work yet.
* (catkin)Exporting to a CSV. Does not work yet.
* Added a close on terminate signal handler.
* (catkin)Added a close on terminate signal handler.
* Made the marker size be set via a parameter to the launch file.
* (catkin)Made the marker size be set via a parameter to the launch file.
* Changed the Callibration data.
* (catkin)Changed the Callibration data.
* The ar_pose listener.
* (catkin)The ar_pose listener.
* Changed the sofie driver to directly safe the ar_pose data.
  We are going to perform experiments and this means that the extra
  data might be useful at a later stage.
* (catkin)Changed the sofie driver to directly safe the ar_pose data.
  We are going to perform experiments and this means that the extra
  data might be useful at a later stage.
* Changed the size of the marker.
* Updated the usb_cam config to work for home camera.
* Added callibration files and launch files.
* Turned off history.
* (catkin)Added some comments and renamed.
* Added some comments and renamed.
* (catkin)The Quaternions were mixed around. Fixed the launch file to log to file instead of screen.
* The Quaternions were mixed around. Fixed the launch file to log to file instead of screen.
* (catkin)Updating the README's.
* Updating the README's.
* Updated the launch file to launch ar_pose and rviz for debugging.
* (catkin)Added arguments to the launch script.
* Added arguments to the launch script.
* Added the Stack formating files.
* (catkin)Organising into a stack instead of separate packages.
* Organising into a stack instead of separate packages.
* Trying to figure out how to start and stop the node.
* Adding simple parameters.
* Added the ROS files.
* Basic driver now works for listening on a channel that broadcasts geometry_msgs.msg.QuaternionStamped messages.
* Working on the listerner that will write to HDFFormat.
* Creating a listerner that can write to sofiehdfformat files.
* Initial commit
* Contributors: Adrian Cooke, Russell Toris, Adrian
