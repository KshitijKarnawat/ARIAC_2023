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
- [OpenCV](https://www.opencv-srf.com/p/introduction.html): Requires both Python and C++ versions. C++ version can be installed from [link](https://www.geeksforgeeks.org/how-to-install-opencv-in-c-on-linux/)
- cv_bridge: Can be installed using - ```sudo apt-get install ros-galactic-cv-bridge```
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

## Run Package

```sh
source /opt/ros/galactic/setup.bash
source <Your workspace>/install/setup.bash
ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa2
ros2 run group3 group3
```

## Package Structure

```txt
.
├─ CMakeLists.txt
├─ LICENSE.md
├─ README.md
├─ config
│  └─ group3_sensors.yaml   # Sensor YAML file for RWA3/RWA4
├─ document
│  ├─ Activity_Diagram_v1.jpg
│  ├─ Class_Diagram_v1.jpg
│  └─ Class_Diagram_v2.jpg  # Modified class diagram
├─ etc
│  ├─ instructions.txt
│  ├─ rwa3.yaml             # Trial YAML file for RWA3
│  └─ rwa4.yaml             # Trial YAML file for RWA4
├─ group3
│  └─ __init__.py
├─ include
│  └─ group3
│     ├─ ariac_competition.hpp
│     ├─ map_poses.hpp
│     ├─ part_type_detect.hpp
│     └─ tray_id_detect.hpp
├─ launch
├─ msg
│  ├─ Part.msg
│  └─ Parts.msg
├─ nodes
│  ├─ .placeholder
│  └─ part_detector.py      # To detect the Part using OpenCV
├─ package.xml
├─ rviz
│  └─ ariac.rviz
└─ src
   ├─ ariac_competition.cpp
   ├─ map_poses.cpp
   ├─ part_type_detect.cpp  
   └─ tray_id_detect.cpp    # To detect the Tray ID using OpenCV

```