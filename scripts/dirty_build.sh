#!/bin/bash

source install/setup.sh

# Build all packages in the workspace
colcon build --continue-on-error --symlink-install

source install/setup.sh
