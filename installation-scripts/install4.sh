#!/bin/bash
user=pses

cd ~
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
cd depends; ./download_debs_trusty.sh
sudo apt-get -y install ocl-icd-opencl-dev
sudo apt-get -y install build-essential cmake pkg-config
sudo apt-get -y install libusb-1.0-0-dev
sudo apt-get -y install libturbojpeg libjpeg-turbo8-dev
sudo apt-get -y install libglfw3-dev
sudo apt-get -y install beignet-dev
sudo apt-get -y install libva-dev libjpeg-dev
sudo apt-get -y install libopenni2-dev
cd ..
mkdir build && cd build
cmake .. -DENABLE_CXX11=ON -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
sudo make install
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
cd /home/$user/catkin_ws/src/
git clone https://github.com/tud-pses/iai_kinect2.git
cd iai_kinect2
rosdep install -r --from-paths .
cd /home/$user/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE="Release"