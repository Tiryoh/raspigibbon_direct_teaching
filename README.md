# raspigibbon_direct_teaching

## About

ROS package suites of Raspberry Pi Gibbon direct teaching

## Requirements

requires the following to run controller on Raspberry Pi 3:

* Ubuntu
  * Ubuntu Xenial 16.04
    * Ubuntu MATE 16.04.1 recomended
* ROS
  * ROS Kinetic
* ROS package
  * [raspberrypigibbon/raspigibbon_ros](https://github.com/raspberrypigibbon/raspigibbon_ros)

## Installation

First, download this repository to your catkin workspace.
e.g.) catkin workspace:`~/catkin_ws/src`

```
git clone https://github.com/Tiryoh/raspigibbon_direct_teaching.git
```

Next, run `catkin_make` and source your new `setup.*sh` file.
If you are using bash, commands should be like this:

```
cd ~/catkin_ws && catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Usage
After installation, launch `direct_teaching.launch`

```
roslaunch raspigibbon_direct_teaching direct_teaching.launch
```

## License

This repository is licensed under the MIT license, see [LICENSE]( ./LICENSE ).

Unless attributed otherwise, everything is under the MIT license.
