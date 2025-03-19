#!/bin/bash

case $# in
0)	echo "Not enough params"
	exit 1
    ;;
esac

for package in "$@" 
do
    rm -rf build/$package install/$package log/
    colcon build --packages-up-to $package --continue-on-error --symlink-install
done

