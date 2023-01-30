#!/bin/bash

############################
#download all dependencies and packages to directory
#https://stackoverflow.com/questions/13756800/how-to-download-all-dependencies-and-packages-to-directory
############################

arg="$1"

mkdir $arg
cd $arg

#apt-get download $(apt-rdepends ${arg} | grep -v "^ ")

# alternative for loop
for PKG in $(apt-rdepends ${arg} | grep -v "^ ");  do apt download $PKG; done

# alternative
#PACKAGES="$1"
#apt-get download $(apt-cache depends --recurse --no-recommends --no-suggests \
#  --no-conflicts --no-breaks --no-replaces --no-enhances \
#  --no-pre-depends ${PACKAGES} | grep "^\w")
