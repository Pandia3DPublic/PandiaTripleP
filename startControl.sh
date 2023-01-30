#!/bin/bash

arg="$1"
if [ "$arg" == "exe" ]
  then
    cd bin/PandiaControl/scripts/
    ./startControl.sh
  else
    echo "Starting Pandia Control"
    cd PandiaControl
    source ControlPythonEnv/bin/activate
    python3 ControlMain.py
    deactivate
fi
