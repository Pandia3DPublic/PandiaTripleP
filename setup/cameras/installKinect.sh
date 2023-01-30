#!/bin/bash

mkdir .tmpPandia
cd .tmpPandia
rm -r *

v=1.4.1

echo "Installing Azure Kinect SDK"
sudo apt-mark unhold k4a-tools
sudo apt-mark unhold libk4a*
sudo apt-get remove -y k4a-tools
sudo apt-get remove -y libk4a*

wget https://packages.microsoft.com/config/ubuntu/18.04/packages-microsoft-prod.deb
sudo dpkg -i packages-microsoft-prod.deb
sudo apt-get update
sudo apt-get install -y k4a-tools=$v libk4a1.4-dev=$v
sudo apt-mark hold k4a-tools
sudo apt-mark hold libk4a*

# increase usb-limit
# see https://importgeek.wordpress.com/2017/02/26/increase-usbfs-memory-limit-in-ubuntu/
sudo sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"/GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"/g' /etc/default/grub
sudo update-grub

# copy udev rule file to access camera without being 'root'
sudo cp ../udev/99-k4a.rules /etc/udev/rules.d/
echo "Done. Please reboot system"

cd ..
rm -r .tmpPandia
