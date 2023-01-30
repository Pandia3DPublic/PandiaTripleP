#!/bin/bash

mkdir .tmpPandia
cd .tmpPandia
rm -r *

v=2.50.0-0~realsense0.6128
vdkms=1.3.18-0ubuntu1

echo "Installing Intel Realsense SDK"
sudo apt-mark unhold librealsense*
sudo apt-get remove -y librealsense*

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install -y librealsense2-dkms=$vdkms librealsense2-utils=$v 
sudo apt-get install -y librealsense2-dev=$v librealsense2-dbg=$v
sudo apt-mark hold librealsense*

cd ..
rm -r .tmpPandia
