#!/bin/bash


#control
cd ../PandiaControl
rm -r ControlPythonEnv
python3 -m venv ControlPythonEnv
source ControlPythonEnv/bin/activate
pip install --upgrade pip
source $which pip
pip install -r requirements.txt
pip install ../3rdParty/Open3D/build/lib/python_package/pip_package/*
deactivate

#workhorse
cd ../PandiaWorkhorse
rm -r WorkhorsePythonEnv
python3 -m venv WorkhorsePythonEnv
source WorkhorsePythonEnv/bin/activate
pip install --upgrade pip
source $which pip
pip install -r requirements.txt
pip install ../3rdParty/Open3D/build/lib/python_package/pip_package/*
deactivate

