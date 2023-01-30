#!/bin/bash

echo "This script installs all essential ubuntu packages for dev"
echo -n "Do you want to continue? (y/n) "
read answer
if [ "$answer" == "${answer#[Yy]}" ] ;then
    echo "Aborting"
    exit
fi

mkdir .tmpPandia
cd .tmpPandia
rm -r *

sudo apt-get update
echo -n "Upgrade installed packages and distro? (y/n) "
read answer
if [ "$answer" != "${answer#[Yy]}" ] ;then
    sudo apt-get upgrade
    sudo apt-get dist-upgrade
fi

echo "Installing essential ubuntu packages"
packages=(\
    build-essential \
    gcc-8 \
    g++-8 \
    lld \
    curl \
    ninja-build \
    dpkg-dev \
    wget \
    htop \
    python3-dev \
    python3-pip \
    python3-venv \
    mlocate \
    net-tools \
    )
sudo apt-get install -y ${packages[@]}

echo "Installing nvidia-cuda-toolkit"
sudo apt-get install -y nvidia-cuda-toolkit

echo "Installing git and git lfs"
sudo apt-get install git -y
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs -y
git lfs install

CmakeVersion=3.21.2
echo "Installing cmake version " $CmakeVersion " using binary files"
wget https://github.com/Kitware/CMake/releases/download/v$CmakeVersion/cmake-$CmakeVersion-linux-x86_64.sh
sudo mkdir /opt/cmake
sudo sh cmake-$CmakeVersion-linux-x86_64.sh --prefix=/opt/cmake --skip-license --exclude-subdir
sudo ln -s /opt/cmake/bin/cmake /usr/bin/cmake

echo "Installing ogre dependencies"
sudo apt-get install -y libgles2-mesa-dev
sudo apt-get install -y libsdl2-dev libxt-dev libxaw7-dev doxygen

echo "Successfully installed all dependencies!"
cd ..
rm -r .tmpPandia
