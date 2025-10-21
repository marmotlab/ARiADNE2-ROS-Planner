# ARiADNE2-ROS-Planner
This repository provides the ROS planner for **HEADER**, which was originally named **ARiADNE2** since it extends our previous work [ARiADNE](https://github.com/marmotlab/ARiADNE-ROS-Planner/tree/main). 

The paper is still ***under review*** so we do not fully release the code yet. 
However, since some people are interested in our terrian segmentation module, we pre-release this module first.
It is modified from open-sourced code (check [here](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/blob/noetic/src/terrain_analysis_ext/src/terrainAnalysisExt.cpp)).


**News:**

**20 Oct 2025** A short version of HEADER was presented at the IROS2025 [active perception workshop](https://activep-ws.github.io/index.html), and it won the Best Paper Award!

**17 Oct 2025** The [preprint version](https://arxiv.org/pdf/2510.15679) of HEADER is available on arxiv!


**Note:** This module is designed for 2.5D planning, so it can not handle multi-floor environments.

<p align="center">
<img width="640" alt="main1" src="https://github.com/user-attachments/assets/fe5b6dbd-79b9-4abc-be21-ca36f4ddb380" />
</p>

## Demo

https://github.com/user-attachments/assets/70f6bb07-0f31-478c-ae21-4e57d38ad9c4

## Usage
First, install ROS [Noetic](http://wiki.ros.org/noetic/Installation) and octomap:
```
sudo apt-get install ros-noetic-octomap
```
Then you can download this repo and compile it.
```
git clone https://github.com/marmotlab/ARiADNE2-ROS-Planner.git
cd ARiADNE2-ROS-Planner
catkin_make -DCMAKE_BUILD_TYPE=Release
```
**Note:** -DCMAKE_BUILD_TYPE=Release is important.

After that, in another workspace, please follow instructions for [CMU Development Environment](https://www.cmu-exploration.com/development-environment) to set up the Gazebo simulation.
Then you may launch the campus environment:
```
source devel/setup.bash 
roslaunch vehicle_simulator system_campus.launch
```
Finally, in another terminal, run:
```
source devel/setup.bash 
roslaunch ariadne2 ariadne2_campus.launch # I commented the planner already
```
Move the robot by clicking the waypoint, you should see a sliding grid map in Rviz.

**Note:** If you would like to test it in different environments, please try to tune parameters in terrian_segmentation.launch to get an idle ground segmentation first.
Besides, this module can not handle negative obstacles, so always be very careful when you deploy it on real robot. We will not be responsible for any damages.

## Author
[Yuhong Cao](https://www.yuhongcao.online)

## Credit
[Development environment](https://www.cmu-exploration.com/development-environment) is from CMU.

[Octomap](https://octomap.github.io/) is from University of Freiburg.

[ChatGPT](https://chatgpt.com/) also contributes some code here and there.



