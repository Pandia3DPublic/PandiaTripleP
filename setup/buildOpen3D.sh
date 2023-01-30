#!/bin/bash

echo -n "(Re)build Open3D? (y/n) "
read answer
if [ "$answer" == "${answer#[Yy]}" ] ;then
    echo "Aborting."
    exit
fi


cd ../3rdParty/Open3D
mkdir build
mkdir install
cd build
rm CMakeCache.txt
cmake -D CMAKE_C_COMPILER=$(which gcc-8) \
      -D CMAKE_CXX_COMPILER=$(which g++-8) \
      -D BUILD_SHARED_LIBS=ON \
      -D BUILD_EXAMPLES=OFF \
      -D BUILD_PYTHON_MODULE=ON \
      -D CMAKE_INSTALL_PREFIX=../install ..
make install -j8
make pip-package

echo "Install script done."
