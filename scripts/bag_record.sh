# if bag dir is not found in home directory, create it
if [ ! -d ~/bag ]; then
  mkdir ~/bag
fi

ros2 bag record /ouster/points /ouster/imu /imu/data /tcpfix /gnss /tf /tf_static -o ~/bag/$(date +%Y%m%d-%H%M%S)