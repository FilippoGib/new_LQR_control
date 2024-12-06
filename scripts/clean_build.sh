#!/bin/bash

rm -rf install log build

# Build all packages in the workspace
colcon build --symlink-install --continue-on-error --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace again to overlay the new packages
source install/setup.sh