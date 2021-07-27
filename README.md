# zed-open-capture-ros

## Update Information
* Add magmoneter data publisher  which publish the magmoneter data at a  rate of 50 HZ.
* Changed the coordinate system. The published sensor data (IMU and Magnetic measurements) use the same coordinate system as the zed-ros-wrapper,
which the X axis  points to the camera front, the Y axis points to the  left side of the camera and the Z axis point to the up side of the camera.
* Changed the unit of measurements. The unit of Gyroscope measurements are in radius, and the magetic measurements are in Telsa.  



## Introduction
A simple ros driver for ZED using zed-open-capture which doesn't depend on CUDA
## Dependencies

* ros (Kinetic/Melodic/Noetic)
* [zed-open-capture](https://github.com/stereolabs/zed-open-capture)

## Usage

Complie:
``` bash
catkin_make
```

Run:
``` bash
source devel/setup.bash
roslaunch zed-open-capture-ros zed_node.launch
```


