#!/bin/bash
executable=$(head -n 1 CurrentExecutable.txt)
buildFolder="build"$executable
echo "Building in "$buildFolder
cd $buildFolder
time ninja
success=$?
if [ $success -eq 0 ] #build successfull
then
	if [ "$executable" = "cppBindings" ]; then
		cd ..
		echo "Opening new terminal for Pandia Workhorse"
		gnome-terminal --window -- ./startWorkhorse.sh
		./startControl.sh
	else
		echo -n "Running Programm "
		echo $executable
		./$executable $@
	fi
else
	echo "Build failed."
fi
