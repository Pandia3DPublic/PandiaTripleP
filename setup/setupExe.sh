#!/bin/bash

# sudo apt install patchelf
# sudo apt install ccache

source ../PandiaControl/ControlPythonEnv/bin/activate
python3 -m pip install nuitka
deactivate

source ../PandiaWorkhorse/WorkhorsePythonEnv/bin/activate
python3 -m pip install nuitka
deactivate
