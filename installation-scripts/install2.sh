#!/bin/bash
user=pses

sudo apt-get -y install qtbase5-dev
cd /home/$user/catkin_ws/src
catkin_init_workspace
cd /home/$user/catkin_ws/
catkin_make
