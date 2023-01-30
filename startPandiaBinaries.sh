#!/bin/bash

cd bin/PandiaWorkhorse/scripts
gnome-terminal --window -- ./startWorkhorse.sh
cd ../../
cd PandiaControl/scripts
./startControl.sh
