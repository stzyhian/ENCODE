# ENCODE: a dEep poiNt Cloud ODometry nEtwork

This repository contains the implementation of ENCODE which estimates the relative 6-DOF pose between two adjacent lidar scans based on end-to-end deep network

_An example of using ENCODE:_
![ptcl](./images/encode.gif)

## Publication

## Pre-trained Models

- [kitti-ResNet18](https://drive.google.com/file/d/1nobeoluR9XjWoHrb_1jo9iMolX9ofLN1/view?usp=sharing)
- [kitti-ResNet50](https://drive.google.com/file/d/1wOVkwGH0MUJdZQQD4Ya_gFOPKeHggiAo/view?usp=sharing)
- [kitti-SqueezeNet](https://drive.google.com/file/d/1aVxP9oQl2YLQ8hr0dyPU_1fXcAXmTS4s/view?usp=sharing)

## Evaluation

Odometry error:
| Dataset         | Translation         | Rotation         |
|-----------------|---------------------|------------------|
| `KITTI 00-06`   | 1.12 %              | 0.49             |
| `KITTI 07-10`   | 1.02 %              | 0.48             |
| `KITTI 10-21`   | 1.86 %              | 0.59             |


| Model           | Translation (00-06) | Rotation (00-06) | Translation (07-10) | Rotation (07-10) |
|-----------------|---------------------|------------------|---------------------|------------------|
| ResNet18        | 1.12 %              | 0.49             | 1.02 %              | 0.48             |
| ResNet50        | 1.14 %              | 0.60             | 1.25 %              | 0.59             |
| SqueezeNet      | 1.19 %              | 0.64             | 1.19 %              | 0.69             |

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

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/stzyhian/ENCODE.git
cd ENCODE
virtualenv -p /usr/bin/python3 venv
source venv/bin/activate
pip install -r requirements.txt
```

## Build
Modify the infer/net.py and set the dataset path and model path
```bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```
## Run

```bash
source devel/setup.bash
roslaunch encode run.launch
rosrun encode net.py
```
