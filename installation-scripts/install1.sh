#!/bin/bash
user=pses

read -p "Skript kann durchaus 1 Stunde benötigen. Fortfahren? (y / n): " -n 1 -r
echo    # (optional) move to a new line
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    exit 1
fi

sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get -y install chromium-browser
sudo apt-get -y purge firefox
sudo apt-get -y install gedit
sudo add-apt-repository -y ppa:webupd8team/sublime-text-3
sudo apt-get update
sudo apt-get -y install sublime-text-installer
sudo apt-get -y install cutecom
sudo apt-get -y install git
sudo apt-get -y install openssh-server
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get -y install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
sudo apt-get -y install python-rosinstall
mkdir -p /home/$user/catkin_ws/src
sudo apt-get -y install --reinstall cmake
