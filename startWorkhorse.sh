#!/bin/bash

arg="$1"
if [ "$arg" == "exe" ]
  then
    cd bin/PandiaWorkhorse/scripts/
    ./startWorkhorse.sh
  else
    echo "Starting Pandia Workhorse"
    cd PandiaWorkhorse
    source WorkhorsePythonEnv/bin/activate
    python3 WorkhorseMain.py
    deactivate
    sleep 10
fi
