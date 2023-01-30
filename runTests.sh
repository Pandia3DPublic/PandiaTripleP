#!/bin/bash
source PandiaControl/ControlPythonEnv/bin/activate
cd tests/control
pytest -rfEP -v
deactivate