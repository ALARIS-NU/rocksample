# Rocksample project

## Installation
 - Despot installation:
```shell
$ cd ~
$ git clone https://github.com/AdaCompNUS/despot.git
$ cd despot
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ../ 
$ make
$ sudo make install
$ sudo apt-get update
$ sudo apt-get install libboost-all-dev
```

 - Project installation:
```shell
$ mkdir -p ~/despot_ws/src
$ git clone https://github.com/ALARIS-NU/rocksample.git ~/despot_ws/src
$ cd ~/despot_ws
$ catkin_make
$ echo 'source ~/despot_ws/devel/setup.bash' >> ~/.bashrc
$ source ~/.bashrc
```

## Usage
 - Run gazebo simulation:
```shell
$ roslaunch rocksample rock_sample.launch
```
 - Run robot controller:
```shell
$ rosrun rocksample rock_planner
```