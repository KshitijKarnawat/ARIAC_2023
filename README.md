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
- [OpenCV](https://www.opencv-srf.com/p/introduction.html): Requires both Python and C++ versions. C++ version can be installed from [link](https://www.geeksforgeeks.org/how-to-install-opencv-in-c-on-linux/). You will also need opencv-contrib for the ArucoDetector functionality. 
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
# For RWA3
ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa3 competitor_pkg:=group3 sensor_config:=group3_sensors
    
# For RWA4
ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa4 competitor_pkg:=group3 sensor_config:=group3_sensors
    
# Launch the group3 node
ros2 launch group3 group3.launch.py
```

Note: If your computer has OpenCV 4.7.0 installed, you might run into issues with cv::ArucoDetector which is meant for older versions of OpenCV like 4.2.0. In such a case, uncomment lines 23-24 and comment out 27-30 in ```tray_id_detect.cpp``` and rerun the demo.

## Package Structure

```txt
.
├─ CMakeLists.txt
├─ LICENSE.md
├─ README.md
├─ config
│  └─ group3_sensors.yaml   # Sensor YAML file for RWA3/RWA4
├─ config
│  └─ group3_sensors.yaml          # Sensor configuration for RWA3/4
├─ document
│  ├─ Activity_Diagram_v1.jpg
│  ├─ Class_Diagram_v1.jpg
│  └─ Class_Diagram_v2.jpg  # Modified class diagram
├─ etc
│  ├─ instructions.txt
│  ├─ rwa3.yaml             # Trial YAML file for RWA3
│  └─ rwa4.yaml             # Trial YAML file for RWA4
│  ├─ Activity_Diagram_v1.jpg      # Activity Diagram for RWA2
│  ├─ Activity_Diagram_v2.jpg      # Activity Diagram for RWA3/4
│  ├─ Class_Diagram_v1.jpg         # Class Diagram for RWA2
│  └─ Class_Diagram_v2.jpg         # Class Diagram for RWA3/4
├─ etc
│  ├─ instructions.txt             # Instructions to run the package for RWA3/4
│  ├─ rwa3.yaml
│  └─ rwa4.yaml
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
│     ├─ ariac_competition.hpp     # Header for AriacCompetition class
│     ├─ map_poses.hpp             # Header for Bin Quadrant Pose class
│     ├─ part_type_detect.hpp      # Header for PartTypeDetect class
│     └─ tray_id_detect.hpp        # Header for TrayIdDetect class
├─ launch
│  └─ group3.launch.py             # Launch file for RWA3/4
├─ msg
│  ├─ Part.msg                     # Message for Type Part
│  └─ Parts.msg                    # Message for Type Parts
├─ nodes
│  ├─ .placeholder
│  └─ part_detector.py      # To detect the Part using OpenCV
│  ├─ .placeholder
│  └─ part_detector.py             # Python script for Part Detection
├─ package.xml
├─ rviz
│  └─ ariac.rviz
└─ src
   ├─ ariac_competition.cpp
   ├─ map_poses.cpp
   ├─ part_type_detect.cpp  
   └─ tray_id_detect.cpp    # To detect the Tray ID using OpenCV
   ├─ ariac_competition.cpp        # Implementation of AriacCompetition class
   ├─ map_poses.cpp                # Implementation of Bin Quadrant Pose class
   ├─ part_type_detect.cpp         # Implementation of PartTypeDetect class
   └─ tray_id_detect.cpp           # Implementation of TrayIdDetect class

```