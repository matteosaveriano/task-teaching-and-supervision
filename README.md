# task-teaching-and-supervision

## Components description
The framework for multimodal teaching and attentional supervision consists of 3 main components:
- _Attentional System (AS)_: Cognitive control mechanism to orchestrate structured tasks execution.
- _Robot Manager (RM)_: Segmentation, motion primitives learning, motor commands generation.
- _LWR Simulator (lwr_rviz_visualization)_: Simulate an LWR IV+ robot and the objects in the scene and visualize them in ```rviz```.

The code is written in C++ and organized into ROS (Robotic Operating System) nodes.

## Software Requirements
The code is developed and tested under _Ubuntu 14.04_ and _ROS Indigo_. All nodes are compiled
using _rosbuild_.

## Installation
This installation guide is for expert ROS (Robotic Operating System) users. Please visit http://www.ros.org/
to get started with ROS.

Before starting, make sure to have the forlders _Attentional_System_, _Robot_Manager_, and
_lwr_rviz_visualization_ in your ROS_PACKAGE_PATH (type in a shell ```echo $ROS_PACKAGE_PATH```).
Make also sure to have Graphviz (```sudo apt-get install graphviz-dev```), Gmp (```sudo apt-get install libgmp-dev```), Eigen (```sudo apt-get install libeigen3-dev```), ARUCO ROS package (```sudo apt-get install
ros-<version>-ar-track-alvar```) installed.
- _Attentional System_:
  - Open a terminal shell
  - Navigate to the catkin workspace folder (typically ```cd ~/catkin_ws```)
  - ```catkin_make seed_segment```
  
_NOTE:_  The compiler assumes that ```libgmp.so.3``` is in ```/usr/lib```. Otherwise, the _Attentional System_ node does not compile, or you will experience a run-time error like: ```error while loading shared libraries: libgmp.so.3: cannot open shared object file: No such file or directory```. This issue can be solved by making a simbolic link to your system version of ```libgmp.so.3```.
  
In a terminal shell type: ```sudo ln -s <path-to-libgmp>/<libgmp-system-version> /usr/lib/libgmp.so.3``` 
For example, assuming that ```libgmp.so.10``` is installed in ```/usr/lib64```, you will type: ```sudo ln -s /usr/lib64/libgmp.so.10 /usr/lib/libgmp.so.3```.

- _Robot Manager_:
  - Open a terminal shell
  - Navigate to the catkin workspace folder (typically ```cd ~/catkin_ws```)
  - ```catkin_make kuka_seed_commands```
  - ```catkin_make lwr_seed_control```

- _LWR Simulator_:
  - Open a terminal shell
  - Navigate to the catkin workspace folder (typically ```cd ~/catkin_ws```)
  - ```catkin_make lwr_rviz_visualization```

## Usage
We prepared a demo version collecting a user demonstration of the prepare coffee task. Data are stored
into the prepare_coffee.bag file.
To reproduce the demo execute the following commands in separate shells:
- ```roscore```
- ```rosparam set use_sim_time true``` (no troubles with old /tf data)
- ```roslaunch lwr_rviz_visualization scene_simulator.launch``` (start the robot and objects simulator)
- ```rosrun lwr_seed_control learnTask``` (start the Robot Manager)
- ```rosrun seed_segment seed_segment``` (start the Attentional System)
- navigate to the folder _lwr_rviz_visualization_ and then type ```rosbag play prepare_coffee.bag --clock``` (playback the recorded data)

After typing these commands, you will have a window showing the task structure updates and a
simulated scene (```/tf``` frames) in Rviz. Note that the frame ```/wsg50_end_link``` represents the end
effector of the robot.

When ```prepare_coffee.bag``` ends you have the full structure and low-level motion primitives learned. The robot starts to execute the task autonomously (see the frame ```/wsg50_end_link``` in Rviz). You have to press the Reset button in Rviz (bottom left) to see the end effector frame moving.

### ROS Topics
The Attentional System (AS) and the Robot Manager (RM) communicate over ROS topics:
- ```/kuka_action``` command (from AS to RM)
- ```/kuka_return``` last executed command or new segment generated (from RM to AS)
- ```/object_dists``` robot-object distances
- ```/seed_stream``` output of the speech recognition (stored in ```prepare_coffee.bag```)

### Notes
You cannot terminate the Attentional System (```seed_segment```) with CTRL+C. Instead, type
```forget(alive)``` and press ENTER.

## Reference
Please acknowledge the authors in any acedemic publication that used parts of these codes.
```
@article{Caccavale2017,
    title = "Kinesthetic Teaching and Attentional Supervision of Structured Tasks in Human-Robot Interaction",
    author = "Caccavale, Riccardo and Saveriano, Matteo and Finzi, Alberto and Lee, Dongheui",
    journal = "Autonomous Robots",
    year = "2017",
    volume = "",
    number = "",
    pages = "",
    month = "",
}
```

## Licence
This repository contains free software: you can redistribute it and/or modify it under the terms of the GNU General Public License version 3 as published by the Free Software Foundation.

This source code is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this code. If not, see http://www.gnu.org/licenses/.
