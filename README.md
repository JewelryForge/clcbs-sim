# CL-CBS-Simulation

This is a gazebo and Songling Hunter simulation of [**Car-Like Conflict Based Search (CL-CBS)**](https://github.com/APRIL-ZJU/CL-CBS). In gazebo, a differential driven model with minimum rotation radius constrain  and in physical environment Hunter robot with Ackerman chassis are used for simulation.

## Requirements
[**osqp**](https://osqp.org/docs/get_started/sources.html#build-from-sources)

```shell
git clone --recursive https://github.com/oxfordcontrol/osqp
cd osqp
mkdir build && cd build
cmake -G "Unix Makefiles" ..
make -j4
sudo make install
```

[**ospq-eigen**](https://github.com/robotology/osqp-eigen)

```shell
git clone https://github.com/JewelryForge/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

ros dependencies

```shell
sudo apt install ros-melodic-joint-state-controller ros-melodic-controller-manager ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-velocity-controllers ros-melodic-map-server xterm
```

## Basic Usage

### Build

```shell
mkdir -p CLCBS/src && cd CLCBS/src
git clone https://github.com/JewelryForge/CL-CBS-Simulation.git
cd ..
catkin_make
```

### Quick Start

```shell
cd <your_ws>
source devel/setup.<your_shell>
roslaunch clcbs_gazebo clcbs_world.launch
roslaunch clcbs_driving tracking.launch map:=src/config/map2.yaml schedule:=src/config/sch2.yaml
```

### Generate your own map and schedule

First write a map.yaml file and generate its schedule sch.yaml according to CL-CBS. Then

```shell
rosrun map_from_yaml setup.sh map.yaml sch.yaml
roslaunch clcbs_driving tracking.launch map:=map.yaml schedule:=sch.yaml
```

 

