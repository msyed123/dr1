![GitHub contributors](https://img.shields.io/github/contributors/msyed123/dr1)
![GitHub repo size](https://img.shields.io/github/repo-size/msyed123/dr1)
![GitHub last commit](https://img.shields.io/github/last-commit/msyed123/dr1)
# DR-1
This is the repo used to maintain the codebase for the DR-1 project at the [Florida Tech 
Dynamic Systems, Controls, and Mechatronics Lab.](https://research.fit.edu/dynamic-systems-and-controls-lab/)  The project utilizes the PX4 flight stack, MAVROS, and OpenCV in order to generate a relative state vector from a target position and generate the necessary commands for the flight computer in order to land on the target. System integration and testing occurs on a Gazebo flight simulator developed by the PX4 group using a Hardware-in-the-Loop approach, and on a Pixhawk equipped hexacopter. Algorithms and code validated on a Raspberry Pi 3B+, however will likely work on a wide variety of companion computers.  

## Packages
This is a ROS project, with several different packages that contain discrete functionality for this project.

### dr1
This folder contains the codes related to launching, controlling, calibrating and positioning the drone. PX4 open source flight control software was used to perform the required tasks of the drone  
