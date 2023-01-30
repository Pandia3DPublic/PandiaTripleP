#!/bin/bash

echo "Installing all camera SDKs"

./installEnsenso.sh
./installKinect.sh
./installPhoxi.sh
./installRealsense.sh
./installZivid.sh

echo "Done installing all camera SDKs. Please reboot the system."
