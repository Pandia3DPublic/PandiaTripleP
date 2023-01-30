#!/bin/bash
#this script is for autostart on login

mkdir -p .Logs

cd PandiaWorkhorse/scripts
gnome-terminal --window -- ./startWorkhorse.sh

cd ../..
cd PandiaControl/scripts
./startControl.sh
