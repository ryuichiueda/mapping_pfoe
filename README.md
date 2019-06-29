# mapping_pfoe

a teach-and-replay package for Raspberry Pi Mouse


## Requirements

This package requires the following:
* Robot
  * Raspberry Pi Mouse
* Laser Rangefinder
  * URG-04LX-UG01
* IMU
  * RT-USB-9AXIS-00
* Ubuntu
  * Ubuntu 18.04 (Ubuntu 18.04 Server recomended)
* ROS
  * Melodic Morenia
* ROS Package
  * urg_node - [urg_node - ROS WiKi](http://wiki.ros.org/urg_node)
  * raspimouse_ros_2 - [ryuichiueda/raspimouse_ros_2](https://github.com/ryuichiueda/raspimouse_ros_2)

## Installation

* Install `raspimouse_ros_2`
    * Check it with the imu
* Install `urg_node` and `move_base`
* Clone this package at the `src` directory of the catkin workspace
* `catkin_make && source ~/catkin_ws/devel/setup.bash`


## Usage

Please place a map in [maps](./maps) and launch nodes with `robot.launch` and `pc.launch` on the Raspberry Pi Mouse side and the PC side respectively.



```
(robot side)$ roslaunch mapping_pfoe robot.launch
(pc side   )$ roslaunch mapping_pfoe pc.launch
```

## License

This repository is licensed under the MIT license, see [LICENSE](./LICENSE).
