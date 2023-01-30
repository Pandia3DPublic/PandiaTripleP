#!/bin/bash

mkdir .tmpPandia
cd .tmpPandia
rm -r *

echo "Installing Phoxi SDK"
# newest version can be found here https://www.photoneo.com/downloads/phoxi-control/
PhoxiInstaller=PhotoneoPhoXiControlInstaller-1.9.0-rc1-Ubuntu20
wget https://photoneo.com/files/installer/PhoXi/1.9/1.9.0-rc1/$PhoxiInstaller.tar.gz
tar -xvf $PhoxiInstaller.tar.gz
chmod +x $PhoxiInstaller.run
sudo ./$PhoxiInstaller.run --accept

cd ..
rm -r .tmpPandia
