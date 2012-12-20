#Change Git Modules to readonly.
cat .gitmodules| sed 's|git@github.com:agcooke/|git://github.com/agcooke/|g' > tmp
mv tmp .gitmodules
#Install Submodules.
git submodule init
git submodule update
#Build Ros
roscd sofiehdfformat_rosdriver/
rosmake
#Enter VirtualEnv
workon sofie
#Install Python Packages.
cd ~/ros/libs/sofiehdformat/
sudo python setup.py develop
cd ~/ros/libs/experimentcontrol/
sudo python setup.py develop