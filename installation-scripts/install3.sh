#!/bin/bash
user=pses

sudo apt-get -y dist-upgrade
sudo apt-get -y install libgles2-mesa
cd /home/$user/catkin_ws/src
rm CMakeLists.txt
git clone https://github.com/tud-pses/PSES-Basis.git .
cd ..
rosdep install -y -r --from-paths .
sudo apt-get -y install ros-kinetic-serial
catkin_make -DCMAKE_BUILD_TYPE="Release"
