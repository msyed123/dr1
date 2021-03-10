![GitHub contributors](https://img.shields.io/github/contributors/msyed123/dr1)
![GitHub repo size](https://img.shields.io/github/repo-size/msyed123/dr1)
![GitHub last commit](https://img.shields.io/github/last-commit/msyed123/dr1)
# DR-1
This is the repo used to maintain the codebase for the DR-1 project at the [Florida Tech 
Dynamic Systems, Controls, and Mechatronics Lab.](https://research.fit.edu/dynamic-systems-and-controls-lab/)  The project utilizes the PX4 flight stack, MAVROS, and OpenCV in order to generate a relative state vector from a target position and generate the necessary commands for the flight computer in order to land on the target. System integration and testing occurs on a Gazebo flight simulator developed by the PX4 group using a Hardware-in-the-Loop approach, and on a Pixhawk equipped hexacopter. Algorithms and code validated on a LattePanda Alpha, however will likely work on a wide variety of companion computers.  

## Install
1. Install Ubuntu 16.04
   1. Make a bootable USB disk with the Ubuntu image as per the instructions here: [https://ubuntu.com/tutorials/create-a-usb-stick-on-ubuntu#1-overview](https://ubuntu.com/tutorials/create-a-usb-stick-on-ubuntu#1-overview)
   2. Boot the LattePanda from the USB drive. In order to do this, rapidly press the F7 button as the device is starting in order to reach the Boot Manager. Select the UEFI OS USB drive that you just created.
   3. Install Ubuntu to the device. It is suggested that a clean install with Ubuntu as the primary operating system is selected.
2. Install ROS Kinetic using these instruction: [http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)
   1. Configure repositories to allow "Restricted", "Universe", and "Multiverse" packages.
   2. Setup Ubuntu sources to accept packages from the ROS package server:

        ```bash
        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        ```
   3. Setup security keys to accept signed software from the ROS package server:

        ```bash
        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
        ```
   4. Update apt index:

        ```bash
        sudo apt-get update
        ```
        
   5. Install ROS Desktop:

        ```bash
        sudo apt-get install ros-kinetic-desktop
        ```

   6. It is suggested to source the ROS environment variables and commands via the bashrc such that the relevant ROS commands are exposed for each terminal:

        ```bash
        echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
        source ~/.bashrc
        ```

   7. Install the necessary dependencies for building ROS packages from source:

        ```bash
        sudo apt-get install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
        ```

   8. Initialize rosdep (the native ROS dependency manager):

        ```bash
        sudo apt-get install python-rosdep
        sudo rosdep init
        rosdep update
        ```

3. Install `catkin_tools` using these instructions: [https://catkin-tools.readthedocs.io/en/latest/installing.html](https://catkin-tools.readthedocs.io/en/latest/installing.html)
   1. Update apt list and install `catkin_tools`:

        ```bash
        sudo apt-get update
        sudo apt-get install python-catkin-tools
        ```

4. Initialize catkin workspace:

     ```bash
     cd ~
     mkdir catkin_workspace
     cd catkin_workspace
     mkdir src
     cd src
     git clone https://github.com/msyed123/dr1.git . --recurse-submodules
     cd ~/catkin_workspace
     catkin init
     echo "source ~/catkin_workspace/devel/setup.bash" >> ~/.bashrc
     ```

5. Install `librealsense` from source: [Librealsense Linux Ubuntu Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
   1. Download `librealsense` sources from Git:

      ```bash
      cd ~
      git clone https://github.com/IntelRealSense/librealsense.git
      ```

   2. Disconnect any connected RealSense camera
   3. Navigate to the `librealsense` directory

      ```bash
      cd librealsense
      ```

   4. Install core packages for building the binary via apt:

      ```bash
      sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
      sudo apt-get install libglfw3-dev
      ```
   5. Adjust UDev rules using the provided bash script.

      ```bash
      ./scripts/setup_udev_rules.sh
      ./scripts/patch-realsense-ubuntu-lts.sh
      ```
   6. Build SDK

      ```bash
      cd ~/librealsense
      mkdir build
      cd build
      cmake ../ -DCMAKE_BUILD_TYPE=Release
      sudo make uninstall && make clean && make -j4 && sudo make install
      ```

6. Install ROS wrapper for `librealsense` (`realsense2-camera`)

     ```bash
     sudo apt-get install ros-kinetic-realsense2-camera
     ```

7.  Install `pcl_ros`, a dependency for `realsense2-camera`

     ```bash
     sudo apt-get install ros-kinetic-pcl-ros
     ```

8. Install `mavros` and `mavros_extras`

     ```bash
     sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
     wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
     sudo bash ./install_geographiclib_datasets.sh
     ```

9. Configure catkin workspace to extend ROS source space

     ```bash
     cd ~/catkin_workspace
     catkin clean
     catkin config --extend /opt/ros/kinetic
     ```

10. Install necessary Python packages:
    
     ```bash
     sudo apt-get install python-pip
     pip install numpy==1.15.0
     pip install scipy==1.2.3 --no-deps
     pip install pyquaternion --no-deps
     pip install filterpy --no-deps
     ```

11. Build catkin workspace

     ```bash
     cd ~/catkin_workspace
     catkin build
     ```

12. Mark all necessary scripts as executable

     ```bash
     chmod +x ~/catkin_workspace/src/dr1/scripts/*.py
     ```

## Packages
This is a ROS project, with several different packages that contain discrete functionality for this project.

### dr1
This folder contains the codes related to launching, controlling, calibrating and positioning the drone. PX4 open source flight control software was used to perform the required tasks of the drone

### VIO
This is a submodule that interfaces an Intel T265 with a Pixhawk 4 Flight Computer. The assumption is that the FCU endpoint is correctly defined in the MAVROS launch file. This is required in order for the correct serialization of the velocity vectors coming from the VIO to the Mavlink endpoint on the FCU.