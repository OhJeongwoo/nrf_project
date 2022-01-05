# Installation

## Environment setting

## ROS melodic
- install ROS melodic via below link
http://wiki.ros.org/melodic/Installation/Ubuntu

- create workspace and build
$ cd && mkdir -p catkin_ws/src
$ cd catkin_ws && catkin_make

## Autoware
You must install ROS before installing autoware. We provide reference link to install Autoware

***https://github.com/Autoware-AI/autoware.ai***

```console
$ sudo apt update
$ sudo apt install -y python-catkin-pkg python-rosdep ros-$ROS_DISTRO-catkin
$ sudo apt install -y python3-pip python3-colcon-common-extensions python3-setuptools python3-vcstool
$ pip3 install -U setuptools
$ cd && mkdir -p autoware.ai/src
$ cd autoware.ai
$ wget -O autoware.ai.repos "https://raw.githubusercontent.com/Autoware-AI/autoware.ai/1.12.0/autoware.ai.repos"
$ vcs import src < autoware.ai.repos
$ rosdep update
$ rosdep install -y --from-paths src --ignore-src --rosdistro melodic
$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Sensor setting

### 1. Novatel GPS and InertialLabs IMU

You need ***inertiallabs pkg*** for running GPS and IMU

$ cd ~/catkin_ws/src && git clone https://us.inertiallabs.com:31443/scm/ins/inertiallabs-ros-pkgs.git

$ cd ~/catkin_ws && catkin_make

The support of the ***FTDI*** is embedded into the Linux core, and usually, no additional settings or configurations are required to work with INS serial ports on Linux. Most probably, the issue you have is caused by access rights.

Please ensure that your current user has access rights to USB serial ports (try to read serial port file configuration, for example: stty -F /dev/ttyUSB0) if not - make sure that every USB-to-serial adapter is assigned to the group dialout and add your current user to this group by issuing the following command as root:

$ usermod -a -G dialout <current-user>

$ usermod -a -G tty <current-user>

If adapters are assigned to root, you need to add a udev rule - an example is shown below: 

$ lsusb

$ cd /etc/udev/rules.d/

and add this line in the file.

***SUBSYSTEM=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", MODE="0664", GROUP="dialout" ***

To find out the vendor ID and product ID of your adapter, you can use lsusb command. The response will show the bus number, device number assigned by the bus, product ID, vendor ID, and the name of attached devices, both vendor and product IDs are in hexadecimal format, an example is shown below (output is cropped to fit the page and explanation text is added on the second line):

you can check success via this commend

$ rosrun inertiallabs_ins il_ins

### 2. PCAN(mobileye)

$ sudo apt install can-utils

you can check mobileye raw can data via below commends.

$ sudo ip link set can0 type can bitrate 500000

$ sudo ifconfig can0 up

$ candum can0

### 3. VLP-16 (3 VLP-16, FrontLeft, FrontRight, RearCenter)

Before installing VLP-16, you have to install autoware. Please refer Installation>Autoware part.

Download zip file via below link
<google drive link>

There are 3 packages and 3 launch files
move 3 packages to <autoware path>/src/autoware/utilities/
and rebuild
$ cd ~/autoware.ai
$ colcon build --packages-select multi_vlp_converter multi_vlp_driver multi_vlp_combiner
$ source install/setup.bash

Recommend write down below commend to ~/.bashrc file
source ~/autoware.ai/install/setup.bash

You should move 3 launch files into ~/autoware.ai/install/runtime_manager/share/runtime_manager/launch_files

you can check velodyne position configuration (position and orientation), ip link, and port number.
ip link and port number must be different each other. You can set them via 192.168.1.20x (x = 1,2,3)
check ip link and port number and compare these parameters with launch file.

All of setting are completed, we can check pointcloud data via below commend
$ roslaunch runtime_manager triple_vlp16.launch

you want to visualize them, open another terminal and run
$ rviz

you have to set fix frame to 'velodyne'.


### 4. Realsense Camera

$ cd ~/catkin_ws/src
$ git clone https://github.com/IntelRealSense/realsense-ros.git
$ sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
$ cd ~/catkin_ws && catkin_make

Please refer below link in detail
https://github.com/IntelRealSense/realsense-ros

