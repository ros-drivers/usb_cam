#Build Ros
#git clone git://github.com/agcooke/roshome.git
roscd roshome/
cat .rosinstall | sed 's|git@github.com:agcooke/|git://github.com/agcooke/|g' > .rosinstall.tmp
mv .rosinstall.tmp .rosinstall
rosws update
cd src/ar_track_alvar
git pull origin groovy-devel
roscd
catkin_make
#Enter VirtualEnv
mkvirtualenv sofie --system-site-packages
workon sofie
#Required to compile PyTables and pythoncard is needed for the GUI.
sudo aptitude install libhdf5-serial-dev pythoncard
#Install Python Packages. (This should be taken care of with pip).
#They will fail but run them again until all dependencies are taken care of.
pip install -e git+git://github.com/uvemas/ViTables#egg=vitables
pip install -e git+git://github.com/agcooke/Sofie-HDF-Format.git#egg=sofiehdfformat --upgrade
pip install -e git+git://github.com/agcooke/ExperimentControl.git#egg=experimentcontrol --upgrade
