#!/bin/bash

echo "Clearing bin folder"
rm -r ../bin/
echo "Building Workhorse"
./buildExe.sh Workhorse
echo "Building Control"
./buildExe.sh Control
