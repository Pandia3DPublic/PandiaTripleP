#!/bin/bash

############################
#download packages to directory
#https://stackoverflow.com/questions/13756800/how-to-download-all-dependencies-and-packages-to-directory
############################

arg="$1"

apt-get download ${arg}

