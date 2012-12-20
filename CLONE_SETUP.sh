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
workon sofie  --system-site-packages
sudo aptitude install vitables
#Install Python Packages. (This should be taken care of with pip).
pip install -e git+git://github.com/agcooke/Sofie-HDF-Format.git
pip install -e git+git://github.com/agcooke/ExperimentControl.git