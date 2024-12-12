#!/bin/bash

source install/setup.sh

# Build all packages in the workspace
colcon build --symlink-install --continue-on-error --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace again to overlay the new packages on top of the existing ones
if [ -n "$BASH_VERSION" ]; then
  source install/setup.bash
elif [ -n "$ZSH_VERSION" ]; then
  source install/setup.zsh
else
  source install/setup.sh
fi