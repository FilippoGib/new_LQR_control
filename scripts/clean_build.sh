#!/bin/bash

rm -rf install log build

# Build all packages in the workspace
colcon build --continue-on-error --symlink-install

# Source the workspace again to overlay the new packages
source install/setup.sh