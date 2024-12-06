#!/bin/bash

source install/setup.sh

# Build all packages in the workspace
colcon build --symlink-install --continue-on-error --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.sh
