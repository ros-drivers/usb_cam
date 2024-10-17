^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package usb_cam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

Forthcoming
-----------
* Merge pull request `#68 <https://github.com/MisoRobotics/usb_cam/issues/68>`_ from MisoRobotics/user/hjoe72/fix/usb-cam-sn
  F3-8491: Fix USB camera SN matching
* Fix USB camera SN matching
  Fix USB cameras SN matching.
* Contributors: Hendry Joe, Zach Zweig Vinegar

1.0.2 (2024-07-18)
------------------
* Merge pull request `#65 <https://github.com/MisoRobotics/usb_cam/issues/65>`_ from MisoRobotics/user/hjoe72/feature/block-manual-reset
  F3-5443: Block manual cycle after init cycle
* Block manual cycle after init cycle
  Block manual camera cycling until the initial cycle
  is completed.
* Merge pull request `#64 <https://github.com/MisoRobotics/usb_cam/issues/64>`_ from MisoRobotics/user/hjoe72/feature/srv-reset-cam
  F3-5443: Add service for resetting the camera exposure
* Add service for resetting the camera exposure
  Add service for resetting camera exposure.
* Merge pull request `#62 <https://github.com/MisoRobotics/usb_cam/issues/62>`_ from MisoRobotics/user/hjoe72/fix/add-wait-before-set-exp
  F3-5443: Add pause before setting the exposure level
* Add pause before setting the exposure level
  Add pause before setting the exposure level during
  the initial manual exposure cycle.
* Merge pull request `#61 <https://github.com/MisoRobotics/usb_cam/issues/61>`_ from MisoRobotics/master
  Backmerge 1.0.1 into develop for flippy-3.0.5-noetic
* Contributors: Hendry Joe, Zach Zweig Vinegar

1.0.1 (2024-07-03)
------------------
* Merge pull request `#57 <https://github.com/MisoRobotics/usb_cam/issues/57>`_ from MisoRobotics/user/hjoe72/fix/usb-cam-param
  F3-2084: Update USB cam param name
* Update USB cam param name
  Update USB cameras param name.
* Merge pull request `#59 <https://github.com/MisoRobotics/usb_cam/issues/59>`_ from MisoRobotics/master
  Backmerge master into develop for flippy-1.0.0
* Contributors: Hendry Joe, Zach Zweig Vinegar

1.0.0 (2024-06-20)
------------------
* Merge pull request `#56 <https://github.com/MisoRobotics/usb_cam/issues/56>`_ from MisoRobotics/user/hjoe72/feature/make-auto-exposure-reset-optional
  F3-2048: Make periodic auto exposure reset optional
* Make periodic auto exposure reset optional
  Make periodic auto exposure reset optional since
  it stops streaming images when resetting.
* Merge pull request `#53 <https://github.com/MisoRobotics/usb_cam/issues/53>`_ from MisoRobotics/master
  Backmerge master into develop for flippy-0.8.4
* Contributors: Hendry Joe, Zach Zweig Vinegar

0.8.4 (2022-10-31)
------------------
* Merge pull request `#51 <https://github.com/MisoRobotics/usb_cam/issues/51>`_ from MisoRobotics/user/hungrh/fix/autobin-auto-reset-exposure-setting
  FSIT-3680: Fix autobin auto reset exposure setting
* Fix autobin auto reset exposure setting
  This commit fixes to reset the exposure setting
  without stopping the capture of the V4L2 buffer.
* Merge pull request `#45 <https://github.com/MisoRobotics/usb_cam/issues/45>`_ from MisoRobotics/user/jhong/feature/add_freq_monitoring
  SFS-8564, SFS-10034: Add new freq monitoring and diagnostic launch files
* SFS-8564: Diag 4 - Add frequency monitoring
  Add frequency monitoring.
* Merge pull request `#50 <https://github.com/MisoRobotics/usb_cam/issues/50>`_ from MisoRobotics/master
  Backmerge master into develop for flippy-0.8.3
* Contributors: Hung Ruo Han, Junpyo Hong, Zach Zweig Vinegar

0.8.3 (2022-10-17)
------------------
* Merge pull request `#46 <https://github.com/MisoRobotics/usb_cam/issues/46>`_ from MisoRobotics/user/hungrh/feature/auto-reset-exposure-setting
  FSIT-3393: Add auto reset exposure setting
* Add auto reset exposure setting
  This commit adds a feature to auto reset the camera
  exposure setting every 1 minute when it is enabled.
* Merge pull request `#48 <https://github.com/MisoRobotics/usb_cam/issues/48>`_ from MisoRobotics/master
  Backmerge master into develop for flippy-0.8.2
* Contributors: Hung Ruo Han, Zach Zweig Vinegar

0.8.2 (2022-10-04)
------------------
* Merge pull request `#43 <https://github.com/MisoRobotics/usb_cam/issues/43>`_ from MisoRobotics/user/hjoe72/feature/shutdown-unresponsive-node
  FSIT-3352: Shutdown stopped node
* Shutdown stopped node
  Instead of stopping the program, properly
  shutdown the node when error occurs. This is useful
  for automatically respawning the node.
* Merge pull request `#41 <https://github.com/MisoRobotics/usb_cam/issues/41>`_ from MisoRobotics/master
  Backmerge master into develop for flippy-0.8.1
* Contributors: Hendry Joe, Zach Zweig Vinegar

0.8.1 (2022-08-22)
------------------
* Merge pull request `#39 <https://github.com/MisoRobotics/usb_cam/issues/39>`_ from MisoRobotics/user/jhong/bugfix/add_dependency
  Add diagnostic_updater dependency
* Add build/exec dependencies.
  Added build/exec dependencies.
* Merge pull request `#38 <https://github.com/MisoRobotics/usb_cam/issues/38>`_ from MisoRobotics/master
  Backmerge master into develop for flippy-0.8.0
* Contributors: Junpyo Hong, Zach Zweig Vinegar

0.8.0 (2022-06-20)
------------------
* Merge pull request `#36 <https://github.com/MisoRobotics/usb_cam/issues/36>`_ from MisoRobotics/master
  Backmerge master into develop for flippy-0.7.2
* Merge pull request `#31 <https://github.com/MisoRobotics/usb_cam/issues/31>`_ from MisoRobotics/user/imjaya/feature/add-frequency-monitoring
  SFS-3757: Add frequency monitoring for USB_Cam Node
* SFS-3757: Add frequency monitoring for USB_Cam Node
  Add frequency monitoring for USB_Cam Node.
* Merge pull request `#33 <https://github.com/MisoRobotics/usb_cam/issues/33>`_ from MisoRobotics/master
  Backmerge master into develop for flippy-0.7.1
* Contributors: Junpyo Hong, Zach Zweig Vinegar

0.7.2 (2022-05-30)
------------------
* Merge pull request `#34 <https://github.com/MisoRobotics/usb_cam/issues/34>`_ from MisoRobotics/user/zzv/hotfix/null-serial
  SFS-7493: Fix not properly handling camera device with null serial
* SFS-7493: Not properly handling camera device with null serial
  Issue discovered due to one of the RealSense cameras having a null
  serial number and causing the driver to throw an exception.
* Contributors: Zach Zweig Vinegar

0.7.1 (2022-04-14)
------------------
* Merge pull request `#29 <https://github.com/MisoRobotics/usb_cam/issues/29>`_ from MisoRobotics/user/imjaya/feature/diagnostics-aggregator
  SFS-4696: Add Heartbeat For Diagnostics Aggregation
* Add Heartbeat for Autobin Camera Nodes
  Add Heartbeat for Autobin Camera Nodes.
* Add Heartbeat for Autobin Camera Nodes
  Added Heartbeat for Autobin
  cameras.
  Add Heartbeat for Autobin Camera Nodes
  Add Heartbeat for Autobin Camera Nodes.
* Merge branch 'user/rsinnet/RAD-171' into develop
* RAD-171: Add labeling workflow
  Add a labeling workflow to apply the `flippy` label.
* Merge pull request `#27 <https://github.com/MisoRobotics/usb_cam/issues/27>`_ from MisoRobotics/master
  Backmerge master into develop for chippy-0.7.0
* Contributors: Jayasurya Sevalur Mahendran, Junpyo Hong, Ryan Sinnet, Zach Zweig Vinegar

0.7.0 (2022-02-14)
------------------
* Merge pull request `#25 <https://github.com/MisoRobotics/usb_cam/issues/25>`_ from MisoRobotics/master
  Backmerge master into develop for chippy-0.6.0
* Merge pull request `#22 <https://github.com/MisoRobotics/usb_cam/issues/22>`_ from MisoRobotics/user/rsinnet/upgrade-catkin-package
  RAD-99: Fix udev dependencies
* Fix udev dependencies
  Add missing dependencies to package.xml.
* Contributors: Ryan Sinnet, Zach Zweig Vinegar

0.6.0 (2022-01-12)
------------------
* Merge pull request `#21 <https://github.com/MisoRobotics/usb_cam/issues/21>`_ from MisoRobotics/user/hruo/fix-mjpeg-pixel-format-new
  SFS-3375: Fix usb_cam loading serial number using MJPEG pixel format
* Fix usb_cam using MJPEG pixel format
  This commits changes the pixel format names between usb_cam and V4L for MJPEG.
* RAD-79: Migrate to Python 3
  Update to work with Python 3 and Noetic.
* Merge remote-tracking branch 'origin/master' into develop
* Merge pull request `#18 <https://github.com/MisoRobotics/usb_cam/issues/18>`_ from MisoRobotics/master
  Merge master back into develop
* Merge pull request `#10 <https://github.com/MisoRobotics/usb_cam/issues/10>`_ from MisoRobotics/user/lrobles/feature/add-ros-parameter-for-tuning-mjpeg
  CRS-782: Add ROS parameter for tuning MJPEG video streaming
* Add ROS parameter for tuning MJPEG video streaming
  MJPEG video frame format can have multiple pixel formats and we need to
  provide a way to allow the user to set up the appropriate one for
  establishing image buffers in the right way. The most common pixel
  formats supported by ffmpeg codec are YUV420P, a 12 bits-per-pixel
  encoding, and YUV422P, a 16 bits-per-pixel encoding. We are adding a
  new integer ROS parameter to control the selection of the both most
  used encodings for MJPEG: 'bits_per_pixel'. Only two values are valid as
  today: 12 and 16; the former is the default.
* Contributors: Hung Ruo Han, Luis Morales Robles, Ryan Sinnet

0.5.2 (2021-10-21)
------------------
* Merge pull request `#16 <https://github.com/MisoRobotics/usb_cam/issues/16>`_ from MisoRobotics/user/hruo/fix/auto-exposure-balance-new
  SFS-2789: Auto-bin exposure camera parametes are not being applied correctly on system startup
* Fix set auto exposure parameter
  Added parameter to auto balance the exposure values
* Contributors: Hung Ruo Han, lpi2020

0.5.1 (2021-10-07)
------------------
* Merge pull request `#15 <https://github.com/MisoRobotics/usb_cam/issues/15>`_ from MisoRobotics/user/araj/feature/add-backlight-compensation
  SFS-2543: Add backlight_compensation parameter to usb parameters
* SFS-2543: Add backlight_compensation parameter to usb parameters
  Add extra parameters to usb camera parameters.
  This parameter help to reduce over exposure and it is necessary to set the
  backlight_compensation parameter to get the expected results.
* Merge pull request `#13 <https://github.com/MisoRobotics/usb_cam/issues/13>`_ from MisoRobotics/release/0.5.0
  release/0.5.0
* Contributors: AbhishekRaj, Ryan Sinnet, Zach Zweig Vinegar

0.5.0 (2021-09-20)
------------------
* Merge pull request `#12 <https://github.com/MisoRobotics/usb_cam/issues/12>`_ from MisoRobotics/user/hruo/feature/add-gamma-parameter-to-usb-camera
  SFS-2431: Add gamma parameter to usb parameters
* Add gamma paramater to usb paramaters
  Add extra parameter to usb camera paramaters
* Merge pull request `#11 <https://github.com/MisoRobotics/usb_cam/issues/11>`_ from MisoRobotics/user/hruo/feature/add-power-frequency-parameter-to-usb-camera-new
  SFS-2413: Add power_line_frequency paramater to usb paramaters
* Add power_line_frequency paramater to usb paramaters
  Add extra parameter to usb camera paramaters
* Merge pull request `#8 <https://github.com/MisoRobotics/usb_cam/issues/8>`_ from MisoRobotics/master
  Merge master back into develop
* Contributors: Hung Ruo Han, Ryan Sinnet, Zach Zweig Vinegar

0.4.0 (2021-07-29)
------------------
* Merge pull request `#6 <https://github.com/MisoRobotics/usb_cam/issues/6>`_ from MisoRobotics/user/hruo/feature/usb-cam-serial-number
  SFS-408: Auto-Bin classification camera drivers
* Fix serial number for usb cam
  This adds a verification to avoid the device file name that doesn't support
  the expected pixel format to be used when more than one device file name have
  the same serial number.
* Add support to serial number for usb cam
  This adds support to match the expected serial number
  (from launch file) and start the node only if they
  match using libudev.
* Merge pull request `#5 <https://github.com/MisoRobotics/usb_cam/issues/5>`_ from MisoRobotics/user/rsinnet/feature/suppress-incompat-warnings
  Suppress incompatibility warnings
* Set log level to error
  This suppresses warnings that blow up non-x86/PPC platforms.
* Suppress warnings
  Supress incompatibility/deprecation warnings.
* Merge pull request `#4 <https://github.com/MisoRobotics/usb_cam/issues/4>`_ from MisoRobotics/user/rsinnet/fix-pixfmt-incompat
  Fix issue with pixfmt compatibility
* Fix issue with pixfmt compatibility
  Change to a deprecated format to work with the buffer size of usb_cam.
* Merge pull request `#1 <https://github.com/MisoRobotics/usb_cam/issues/1>`_ from ros-drivers/develop
  Merge latest from upstream
* Merge pull request `#124 <https://github.com/MisoRobotics/usb_cam/issues/124>`_ from k-okada/add_noetic
  add noetic .travis.yml
* add noetic .travis.yml
* 0.3.6
* update CHANGELOG
* Merge pull request `#71 <https://github.com/MisoRobotics/usb_cam/issues/71>`_ from ros-drivers/fix_L
  add AV\_ to PIX_FMT\_* for X,Y
* .travis.yml: udpate to trusty
* add AV\_ prefix to PIX_FMT\_* for X,Y
* 0.3.5
* update CHANGELOG
* Merge pull request `#69 <https://github.com/MisoRobotics/usb_cam/issues/69>`_ from k-okada/add_ros_orphaned_packages_maintaneres_to_package_xml
  add ROS Orphaned Package Maintainers to maintainer tag
* add ROS Orphaned Package Maintainers to maintainer tag
* Merge pull request `#58 <https://github.com/MisoRobotics/usb_cam/issues/58>`_ from AutonomosGmbH-DaS/kinetic
  support for Kinetic / Ubuntu 16.04
* replace use of deprecated functions in newer ffmpeg/libav versions
  ffmpeg/libav 55.x (used in ROS Kinetic) deprecated the avcodec_alloc_frame.
* Merge pull request `#45 <https://github.com/MisoRobotics/usb_cam/issues/45>`_ from groove-x/develop
  Add "grey" pixel format.
* Add grey scale pixel format.
* Merge pull request `#44 <https://github.com/MisoRobotics/usb_cam/issues/44>`_ from ludusrusso/develop
  add start/stop capture services
* better management of start/stop
* up package.xml
* add capture service
* Merge pull request `#40 <https://github.com/MisoRobotics/usb_cam/issues/40>`_ from ezavesky/develop
  - fix bug for byte count in a pixel (3 bytes not 24 bytes)
* - fix bug for byte count in a pixel (3 bytes not 24 bytes)
* Contributors: Daniel Seifert, Eric Zavesky, Hung Ruo Han, Kei Okada, Ludovico Russo, Russell Toris, Ryan Sinnet, Zach Zweig Vinegar, honeytrap15

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
