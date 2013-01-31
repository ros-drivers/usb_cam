#Build Ros
roscd roshome/
source devel/setup.sh 
roswc update
catkin_make
#Enter VirtualEnv
workon sofie  --system-site-packages
sudo aptitude install vitables
#Install Python Packages. (This should be taken care of with pip).
pip install -e git+git://github.com/agcooke/Sofie-HDF-Format.git
pip install -e git+git://github.com/agcooke/ExperimentControl.git
