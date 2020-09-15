# ENCODE: a dEep poiNt Cloud ODometry nEtwork

This repository contains the implementation of ENCODE which estimates the relative 6-DOF pose between two adjacent lidar scans based on end-to-end deep network

_An example of using ENCODE:_
![ptcl](./images/encode.gif)

## Publication

## Pre-trained Models

- [kitti-resnet18](https://drive.google.com/file/d/1nobeoluR9XjWoHrb_1jo9iMolX9ofLN1/view?usp=sharing)

## Evaluation

Odometry error:
| Dataset         | Translation      | Rotation            |
|-----------------|------------------|---------------------|
| `KITTI 00-06`   | 1.12 %           | 0.0049 [deg/m]      |
| `KITTI 07-10`   | 1.02 %           | 0.0048 [deg/m]      |
| `KITTI 10-21`   | 1.86 %           | 0.0059 [deg/m]      |

## Dependencies

Ubuntu 64-bit 18.04.

* ROS Melodic (desktop-full)
* torch==1.3.0
* torchvision==0.2.2.post3
* numpy==1.14.5
* matplotlib==2.2.3
* vispy==0.5.3
* opencv_python==4.1.0.25
* opencv_contrib_python==4.1.0.25
* Pillow==6.1.0
* scipy==1.5.2
* PyYAML==5.1.1

## Build
```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```
## Run

```bash
source devel/setup.bash
roslaunch encode run.launch
rosrun encode net.py
```