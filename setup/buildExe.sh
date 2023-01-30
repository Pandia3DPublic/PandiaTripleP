#!/bin/bash

if [ $# -eq 0 ]
  then
    echo "No arguments supplied. Arguments are: Control, Workhorse"
    exit
fi

arg="$1"
if [ "$arg" != "Control" ] && [ "$arg" != "Workhorse" ]
  then
    echo "Unknown arguments supplied. Arguments are: Control, Workhorse"
    exit
fi

if [ "$arg" == "Control" ]
  then
    folder=PandiaControl
    main=ControlMain.py
    env=ControlPythonEnv
    out=../bin/PandiaControl
    distfolder=ControlMain.dist
  else
    folder=PandiaWorkhorse
    main=WorkhorseMain.py
    env=WorkhorsePythonEnv
    out=../bin/PandiaWorkhorse
    distfolder=WorkhorseMain.dist
fi

rm -r $out
cd ../$folder
source $env/bin/activate
echo "$env/bin/activate"

# #####################################################
# build with virtual env dependend packages, only local and cannot move to another pc
# notes for deployment: venvs arent meant to be moved to another pc. they have to be created on the other pc so that paths are correct.
# moreover the resulting bin has absolute paths to the packages from the venv. Therefore we should really use Standalone mode for deployment.
# #####################################################
# out=.
# python3 -m nuitka --include-package=core --output-dir=$out --remove-output $main
# cp ../buildcppBindings/cppBindings.cpython-38-x86_64-linux-gnu.so $out


# #####################################################
# build as standalone, wip, should use opencv_python==4.5.3.56 (higher versions cause problem with missing opencv config.py)
# #####################################################
python3 -m nuitka --standalone --enable-plugin=numpy --python-flag=no_site --remove-output --output-dir=$out $main
mv $out/$distfolder/* $out/
rm -d $out/$distfolder

echo "Copying files"
cp -R core $out/
cp *.json $out/
cp ../setup/deployment_all/* ../bin/
cp -R ../resources/ExampleModels ../bin/
mkdir $out/scripts
if [ "$arg" == "Control" ]
  then
    cp ../setup/deployment_control/* $out/scripts/
    rm $out/core/db/db_*
    echo "Deleting 3D Models and database"
    rm $out/core/static/3dModels/*
    rm $out/core/database.db
  else
    cp ../buildcppBindings/cppBindings.cpython-38-x86_64-linux-gnu.so $out/
    cp -R ../buildcppBindings/cppLibs $out/
    cp -R functionConfigs/ $out
    cp -R ../setup/deployment_workhorse/* $out/scripts/
    cp -R ../setup/cameras $out/scripts/
    mkdir -p $out/resources/quality
    mkdir -p $out/resources/volume
fi
echo "Deleting py files"
find $out/core -type f -name '*.py' -delete
find $out/core -type f -name '*.pyc' -delete
find $out/core -type d -name '__pycache__' -delete

deactivate

cd ../setup
echo "Done."
