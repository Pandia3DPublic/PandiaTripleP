#!/bin/bash
[ "$UID" -eq 0 ] || exec sudo "$0" "$@"

cd ../cppLibs
cvpath=$(find $(pwd) -type d -name 'opencv' | head -1)/

echo "This script adds OpenCV librariy path to LD_LIBRARY_PATH"
echo -n "Path is : "
echo $cvpath

echo -n "Proceed? (y/n) "
read answer
if [ "$answer" == "${answer#[Yy]}" ] ;then
    echo "Aborting"
    exit
fi

sudo echo $cvpath > /etc/ld.so.conf.d/OpenCVFixBin.conf
sudo ldconfig
