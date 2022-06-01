# DV ROS Aedat4

This project provides a stand-alone executable for converting aedat4 files into rosbags.

## Usage

Call the compiled executable and pass the path to an aedat4 file to convert. Available parameters:
```
Usage: ./devel/lib/dv_ros_aedat4/convert_aedat4 [OPTIONS]

Options:
  -h,--help                   Print this help message and exit
  -i,--input TEXT:FILE REQUIRED
                              Input aedat4 file
  -o,--output TEXT            Output aedat4 file
  -n,--namespace [/recording]
                              Topic namespace
  -f,--force-overwrite
  -v,--verbose
```
