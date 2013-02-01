#!/bin/bash
rosws update
catkin_make
source devel/setup.sh 
workon sofie
pip install -e git+git://github.com/agcooke/Sofie-HDF-Format.git#egg=sofiehdfformat --upgrade
pip install -e git+git://github.com/agcooke/ExperimentControl.git#experimentcontrol --upgrade