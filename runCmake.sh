#!/bin/bash
echo "Running Cmake Script"
options="cppBindings Test"
validOptions="false"
# echo $1
for i in $options; do
  # echo $i
  if [ $i = $1 ]; then
    validOptions="true"
  fi 
done

if [ $# -ne 1 ]; then
  validOptions="false"
fi
if [ "$validOptions" = "false" ]; then
  echo -n "Error: Please supply the name of the Executable you want to build. Options are: "
  echo $options
  exit 0
fi

path="build"$1
echo "Calling Cmake with Argument" $1
echo "Building inside "$path
cd $path
success=$?
if [ $success -ne 0 ] 
then
  mkdir $path
  cd $path
  echo "### No building folder with the name" $path "exists. Folder created ###"
fi
rm -r cppLibs
rm *linux-gnu.so
cmake -G Ninja \
  -D EXECUTABLE=$1 \
  -D CMAKE_BUILD_TYPE=RelWithDebInfo \
  -D CMAKE_SUPPRESS_DEVELOPER_WARNINGS=ON \
  -D CMAKE_CXX_EXTENSIONS=OFF \
  -D CMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -D CMAKE_C_COMPILER=$(which gcc-8) \
  -D CMAKE_CXX_COMPILER=$(which g++-8) \
  -D CMAKE_LINKER=$(which lld) \
  -D CMAKE_CXX_FLAGS="-fuse-ld=lld" ..
cd ..
echo $1 > CurrentExecutable.txt

