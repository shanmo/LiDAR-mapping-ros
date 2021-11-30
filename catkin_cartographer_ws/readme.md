## platform 

- ubuntu 18.04
- ROS melodic 

## cartographer installation 

```
$ sudo apt-get update
$ sudo apt-get install -y python-wstool python-rosdep ninja-build

$ mkdir catkin_cartographer_ws
$ cd catkin_cartographer_ws
$ wstool init src
$ wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
$ wstool update -t src

src/cartographer/scripts/install_proto3.sh
sudo apt-get install python-rospkg
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
sudo chmod +x src/cartographer/scripts/install_abseil.sh
./src/cartographer/scripts/install_abseil.sh
catkin_make_isolated --install --use-ninja
```

## how to run demo 

```
source install_isolated/setup.bash

wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_3d/with_intensities/b3-2016-04-05-14-14-00.bag

roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag
```

