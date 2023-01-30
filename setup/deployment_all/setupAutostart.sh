#!/bin/bash

mkdir ~/.config/autostart/
cp pandia.desktop ~/.config/autostart/

ExecFolder=$(pwd)
ExecFile=$(pwd)/startPandiaBinaries.sh
IconFile=$(pwd)/PandiaControl/core/static/icons/favicon.png

echo $ExecFolder
echo $ExecFile
echo $IconFile

Exec="gnome-terminal --window -- bash -c \"cd ${ExecFolder} \&\& ${ExecFile}\; exec bash\""

sed -i "s|Exec=|Exec=${Exec}|g" ~/.config/autostart/pandia.desktop
sed -i "s|Icon=|Icon=${IconFile}|g" ~/.config/autostart/pandia.desktop
