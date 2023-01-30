#!/bin/bash

sudo apt remove -y freecad-python3 freecad
sudo apt autoremove -y
sudo snap remove freecad --purge
sudo add-apt-repository -y ppa:freecad-maintainers/freecad-stable
sudo add-apt-repository -y universe
sudo apt update
sudo apt install -y calculix-ccx graphviz freecad-python3 freecad
