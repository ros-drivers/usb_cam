#!/bin/bash
git pull origin master
cat .rosinstall | sed 's|git@github.com:agcooke/|git://github.com/agcooke/|g' > .rosinstall.tmp
rosws update
catkin_make
source devel/setup.sh 
workon sofie
pip install -e git+git://github.com/agcooke/Sofie-HDF-Format.git#egg=sofiehdfformat --upgrade
pip install -e git+git://github.com/agcooke/ExperimentControl.git#experimentcontrol --upgrade