# ARIAC 2023 - Group 3

## Overview

This repository contains the code for the Course ENPM663 - Building a Manufacturing Robotic Software System
The course will focus on the development of a simulation-based control system that will address challenges presented in the Agile Robotics for Industrial Automation Competition(ARIAC)

## Team Members

- Sanchit Kedia (UID: 119159620)
- Adarsh Malapaka (UID: 118119625)
- Tanmay Haldankar (UID: 119175460)
- Sahruday Patti (UID: 118218366)
- Kshitij Karnawat (UID: 119188651)

## Dependencies

- [ROS2(Galactic)](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
- [ARIAC 2023 Workspace](https://github.com/usnistgov/ARIAC)
- [Ubuntu 20.04 LTS](https://releases.ubuntu.com/focal/)

## Build Package

```sh
source /opt/ros/galactic/setup.bash
source <Your workspace>/install/setup.bash
cd <Your ROS2 workspace src folder>
git clone https://github.com/Sanchitkedia/ARIAC_2023.git group3
cd ..
rosdep update --include-eol-distros
rosdep install --from-paths src -y --ignore-src
colcon build --packages-select group3
```
