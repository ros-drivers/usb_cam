Ros Home - Sofie Bicycle Setup.
=======

This document describes how to install the bicycle stability test bench on an Ubuntu machine.

The system is built using (ROS)[http://www.ros.org/wiki/], SOFIE HDF FORMAT. 
There is also a package created for working with experiments.
### Download and install (Ubuntu)[http://releases.ubuntu.com/precise/]

1. Create usb install disk from the download iso: The best Ubuntu version to use for now is
Ubuntu 12.04.

     $sudo usb-disk-creator

2. Boot the USB stick in the new computer and install from the menu on the new computer.
3. Setup EDUROAM: (UT Eduroam setup)[http://www.utwente.nl/icts/en/handleidingen/netwerk/]

### Download and install ROS
Follow the instructions on the ROS website to add the repository for ubuntu.
ROS (install)[http://www.ros.org/wiki/fuerte/Installation/Ubuntu]

Install it from repository (groovy is stable).

    $ sudo aptitude install ros-groovy-desktop ros-groovy-image-view ros-groovy-rosserial
    
    $ sudo aptitude install git openssh-server


Edit your ~/.bashrc (this is common to all ROS installs):

     source /opt/ros/groovy/setup.bash
     export ROS_WORKSPACE=/home/${USER}/roshome/
     export ROS_PACKAGE_PATH=$ROS_WORKSPACE:$ROS_PACKAGE_PATH
     export ROS_HOSTNAME=127.0.0.1
     export ROS_MASTER_URI=http://127.0.0.1:11311

Clean and install the correct packages:

    $ sudo rosdep init
    $ rosdep update
    
### Download and install SOFIE ROS.

	$ cd ~/
	$ git clone git://github.com/agcooke/roshome.git
	$ roscd
	$ rosws update
	$ cd src/ar_track_alvar
Get the correct groovy branch for ar_track_alvar. This could change later.
	$ git pull origin groovy-devel
	$ roscd
	$ rm -r devel/
	$ catkin_make
	$ source devel/setup.sh 
	

### Install the SOFIE PYTHON packages.

Python setup. Sofie uses pytables, so the easiest way to get all the dependencies to install is
to add the utility program vitables.

    $ sudo aptitude install vitables pythoncard

Then use virtualenv to manage the installation. Install: 
[http://www.doughellmann.com/projects/virtualenvwrapper/]

    $ mkvirtualenv sofie --system-site-packages
    $ pip install -e git+git://github.com/agcooke/Sofie-HDF-Format.git#egg=sofiehdfformat --upgrade
	$ pip install -e git+git://github.com/agcooke/ExperimentControl.git#egg=experimentcontrol --upgrade

### Install the Promove GUI information.
Now install IMU software to work with the Intertia Technology devices:
Edit the apt file to add a debian package:

    $sudo vi /etc/apt/sources.list

Then add the following lines

    def file:///home/${USER}/ExtraDebPackages/ /

Then make the directory 

    $ mkdir /home/${USER}/ExtraDebPackages/

Copy the latest promovegui debian file to this directory.
Then clone bashsnippets:

    $ mdir ~/workspace
    $ cd ~/workspace
    $ git clone git://github.com/agcooke/bashsnippets.git
    $ cp bashsnippets/deb/CreatePackages /home/${USER}/ExtraDebPackages/
    $ sudo sh CreatePackages
    $ sudo aptitude update
    $ sudo aptitude install promovegui

The user needs write permission on the serial ports:

    $sudo usermod ${USER} -G dialout

You might have to log out and back in again to get the correct
permissions on the /dev/ttyUSB0 or /dev/ttyUSB1,2,3,4,... that PromoveGUI uses.

Now everything is installed and ready to use.

    $ workon sofie
    $ experiment-control.py -h