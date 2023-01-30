#!/bin/bash
[ "$UID" -eq 0 ] || exec sudo "$0" "$@"

cd ../3rdParty/opencv
cvpath=$(find $(pwd) -type d -name 'lib' | head -1)/

echo "This script adds OpenCV librariy path to LD_LIBRARY_PATH"
echo -n "Path is : "
echo $cvpath

echo -n "Proceed? (y/n) "
read answer
if [ "$answer" == "${answer#[Yy]}" ] ;then
    echo "Aborting"
    exit
fi

sudo echo $cvpath > /etc/ld.so.conf.d/OpenCVFix.conf
sudo ldconfig
