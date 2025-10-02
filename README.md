# FBG_Hybrid_Modeling

## Overview
This ROS package provides a hardware interface for Fiber Bragg Grating (FBG) shape sensors used in a Continuum Dexterous Manipulator (CDM). It enables real-time acquisition of FBG sensor data and integrates with a stereo-camera setup to generate ground-truth shape information for validation.
The stereo-camera module is implemented using OpenCV, which provides a graphical interface for adjusting video-capture properties and includes utilities for applying color masks to markers attached to the CDM. These features allow synchronized collection of FBG wavelength data and 3D pose data from visual tracking.
This repository will include:

- Code for collecting synchronized FBG sensor data and stereo-camera ground truth

- OpenCV utilities for camera calibration, property tuning, and marker color filtering

- Scripts for dataset generation and preprocessing for CDM shape reconstruction tasks

## Dependencies
This package depends on:

- **ROS**
  - The package is tested on Ubuntu 20.04 with ROS noetic
- **OpenCV** 
- **NumPy / SciPy**
- **cisst framework (JHU)**  
  - [cisst](https://github.com/jhu-cisst/cisst.git) – C++ library for surgical robotics and real-time control
- **BIGSS Lab repositories**
  - [util (private)](https://git.lcsr.jhu.edu/bigss/util.git) – This repository is private. Access requires credentials from the BIGSS lab.
 
## Instruction
- Clone the repository to a directory
- Build the package
```
cd DIR_WS
source /opt/ros/noetic/setup.bash
catkin build
```
- Start data collection
```
roslaunch fbg_hybrid_modeling testmtsTaskFBGInterrogator.launch
```


