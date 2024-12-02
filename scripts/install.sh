#!/bin/bash

BASE_DIR=$(pwd)

install_ros2_humble() {
    
    # update and upgrade packages and autoremove the unused pack 
    sudo apt update && sudo apt upgrade && sudo apt autoremove
    echo "finish update, upgrade and autoremove"

    # add the config for the ros2 humble
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    # refresh the package with the ros2-humble archive
    sudo apt update && sudo apt upgrade
    sudo apt install ros-humble-desktop python3-argcomplete
    sudo apt install python3-colcon-common-extensions
    echo "finish ros2-humble installation"

    sudo apt install ros-dev-tools
}

install_libicp() {
    
    SETUP_LIBICP="$BASE_DIR/setup/libicp"
    sudo apt install build-essential libboost-system-dev libboost-thread-dev libboost-program-options-dev libboost-test-dev libboost-filesystem-dev

    cd "$SETUP_LIBICP"
    rm -r "$SETUP_LIBICP"/build
    mkdir "$SETUP_LIBICP"/build && cd "$SETUP_LIBICP"/build
    cmake ..
    make -j${nproc}
    cd ..

    sudo cp "$SETUP_LIBICP"/build/libicp.so /usr/lib/libicp.so
    if [ ! -d "/usr/include/icp" ]; then
        sudo mkdir /usr/include/icp
    fi
    sudo cp "$SETUP_LIBICP"/src/*.h /usr/include/icp/

    sudo rm -rf "$SETUP_LIBICP"/build
}

: '
install_opencv() {
    # TODO: check correct opencv for jetson 6.1 (also check if it is already installed)
}
'

#check if ros2-humble is installed
#for now skip
if [ ! -d "/opt/ros/humble/" ]; then
   echo "don't found ros2-humble installation"
    install_ros2_humble
fi

# # check if libicp is installed
# if [ ! -f "/usr/lib/libicp.so" ]; then
#     install_libicp
#     echo "libicp installed"
# fi

# # check if opencv is installed
# if [ ! -L "/usr/include/opencv" ]; then
#     install_opencv
# fi

# change from foxy to humble
sudo apt install -y libgps-dev \
 libyaml-cpp-dev \
 ros-humble-camera-info-manager \
 ros-humble-diagnostic-updater \
 ros-humble-ackermann-msgs \
 ros-humble-nav2-costmap-2d \
 ros-humble-nav2-bt-navigator \
 ros-humble-joint-state-publisher \
 ros-humble-urdf-tutorial \
 ros-humble-xacro \
 libpcap-dev \
 libpcl-dev \
 ros-humble-pcl-conversions \
 libgoogle-glog-dev \
 ros-humble-cv-bridge \
 ros-humble-image-transport \
 ros-humble-tf-transformations
