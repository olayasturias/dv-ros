# DV ROS 

ROS drivers and sample nodes for iniVation cameras and DV software infrastructure. 


## Installation on Ubuntu OS

The code depends on DV software libraries, these libraries need to be installed for the ROS nodes to compile.
Enable the appropriate iniVation PPA depending on your Ubuntu distribution:

* For Ubuntu 18.04:
```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo add-apt-repository ppa:inivation-ppa/inivation-bionic
sudo apt update
sudo apt install dv-processing dv-runtime-dev
```

* For Ubuntu 20.04:
```
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt update
sudo apt install dv-processing dv-runtime-dev
```

The project is build using catkin tools, run the following commands from your catkin workspace:
```
# Run in your catkin workspace root directory
cd src
git clone https://this_repo
cd ..
catkin build
```

After the build, source your environment to load the information about the new packages, connect your camera
to the computer and validate the build by running visualization sample:
```
source devel/setup.bash
roslaunch dv_ros_visualization event_visualization.launch
```

You should see a preview of events coming from the iniVation camera connected to your computer.

## Repository structure

The repository contains multiple projects:
* dv_ros_msgs - Basic data types for the cameras
* dv_ros_messaging - C++ headers required to use dv-processing in ROS nodes
* dv_ros_capture - Camera driver node (supports live camera data streaming and aedat4 file playback)
* dv_ros_accumulation - Event stream to frame accumulation
* dv_ros_aedat4 - Convert aedat4 files to rosbags
* dv_ros_runtime_modules - DV runtime modules for integration with ROS
* dv_ros_visualization - Simple visualization of events
* dv_ros_tracker - Lucas-Kanade feature trackers for event and image streams
