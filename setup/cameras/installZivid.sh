#!/bin/bash

mkdir .tmpPandia
cd .tmpPandia
rm -r *

echo "Installing Zivid Software"

arg="$1"
if [ "$arg" != "basic" ]
  then
	echo "Installing Zivid required OpenCL drivers (intel packages)"
	# see https://support.zivid.com/latest/getting-started/software-installation/gpu/install-opencl-drivers-ubuntu.html
	# newest version here: https://github.com/intel/compute-runtime/tags
	
	sudo apt-get remove -y intel-igc*
	sudo apt-get remove -y intel-level-zero*
	sudo apt-get remove -y intel-opencl*
	sudo apt-get remove -y intel-gmmlib*
	
	mkdir opencl
	cd opencl
	wget https://github.com/intel/intel-graphics-compiler/releases/download/igc-1.0.11378/intel-igc-core_1.0.11378_amd64.deb
	wget https://github.com/intel/intel-graphics-compiler/releases/download/igc-1.0.11378/intel-igc-opencl_1.0.11378_amd64.deb
	wget https://github.com/intel/compute-runtime/releases/download/22.23.23405/intel-level-zero-gpu-dbgsym_1.3.23405_amd64.ddeb
	wget https://github.com/intel/compute-runtime/releases/download/22.23.23405/intel-level-zero-gpu_1.3.23405_amd64.deb
	wget https://github.com/intel/compute-runtime/releases/download/22.23.23405/intel-opencl-icd-dbgsym_22.23.23405_amd64.ddeb
	wget https://github.com/intel/compute-runtime/releases/download/22.23.23405/intel-opencl-icd_22.23.23405_amd64.deb
	wget https://github.com/intel/compute-runtime/releases/download/22.23.23405/libigdgmm12_22.1.3_amd64.deb
	sudo dpkg -i *.deb
	# verify opencl install
	sudo apt install -y clinfo
	echo "Verifying OpenCL install ..."
	/usr/bin/clinfo -l
	cd ..
fi

# install zivid software
# see https://support.zivid.com/latest/getting-started/software-installation.html
sudo apt-mark unhold zivid-studio zivid-telicam-driver zivid-tools zivid
sudo apt-get remove -y zivid-studio zivid-telicam-driver zivid-tools zivid

sudo apt install -y udev libatomic1 libc6 libgcc1
mkdir zivid
cd zivid
wget https://www.zivid.com/hubfs/softwarefiles/releases/2.7.0+e31dcbe2-1/u20/zivid-telicam-driver_3.0.1.1-3_amd64.deb
wget https://www.zivid.com/hubfs/softwarefiles/releases/2.7.0+e31dcbe2-1/u20/zivid_2.7.0+e31dcbe2-1_amd64.deb
wget https://www.zivid.com/hubfs/softwarefiles/releases/2.7.0+e31dcbe2-1/u20/zivid-studio_2.7.0+e31dcbe2-1_amd64.deb
wget https://www.zivid.com/hubfs/softwarefiles/releases/2.7.0+e31dcbe2-1/u20/zivid-tools_2.7.0+e31dcbe2-1_amd64.deb
sudo dpkg -i *.deb
sudo apt-mark hold zivid-studio zivid-telicam-driver zivid-tools zivid
cd ../..

mkdir --parents $HOME/.config/Zivid/API
cp zivid/Cameras.yml $HOME/.config/Zivid/API/

rm -r .tmpPandia
echo "Done. You may need to re-plug the Zivid camera from PC"
