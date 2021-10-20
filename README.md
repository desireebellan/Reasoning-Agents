# Human in the loop
This repository consitutes the final project for the course "Reasoning Agents" (Master course Artificial Intelligence and Robotics, Sapienza University, Rome) held by prof. Giuseppe De Giacomo during the accademic year 2020/2021. 
## Introduction

## Preliminary Theory
## Problem formulation
[[1]](#1)

*Here will go the explaination of the most important part of the code, what we took from pre existing works and what we implemented*

The whole implementation revolves around the mix-initiative controller:

<img src="https://render.githubusercontent.com/render/math?math=u\triangleq u_r(x,\pi_s, \pi_g)%2B\kappa(x,\Pi)u_h(t)">

where the total controller is the sum of the navigation control and the user control, weighted by a gain of the form:

<img src="https://render.githubusercontent.com/render/math?math=\kappa(x,\Pi)\triangleq \frac{\rho(d_t-d_s)}{\rho(d_t-d_s) %2B \rho(\epsilon+d_s-d_t)}">

## Simulations

## Results
*graphs, gifs, photos...*
## References
<a id="1">[1]</a> 
Guo, M., Andersson, S. et al..
"Human-in-the-Loop Mixed-Initiative Control Under Temporal Tasks"
2018 IEEE International Conference on Robotics and Automation (ICRA)(2018).
## Instructions
* Install ROS [here](http://wiki.ros.org/ROS/Installation)(choose the release that best fits your distro). We've been using ROS noetic on ubuntu 20.04 for TURTLEBOT and ROS melodic on ubuntu 18.04 for TIAGO.
### TURTLEBOT simulations

* Clone the turtlebot3, turtlebot3_msg and turtlebot3_simulations packages inside ~/catkin_ws/src
```
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b -{melodic|noetic}-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b -{melodic|noetic}-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
* Clone this repository and build 
```
cd ~/catkin_ws/src
git clone ...
cd ..
catkin_make
```
* Inside the directory ~/catkin_ws source the environment defined in the prrvious step
```
cd ~/catkin_ws
source ~/.bashrc
```
* Run the following instructions on different terminals
```
roslaunch turtlebot3_gazebo turtlebot3_gazebo_wolrd wolrd:=
```
### Solved Issues
* If turtlebot3_teleop does not work with gazebo, try to reinstall gazebo_ros_pkgs
```
sudo apt-get install ros-{melodic|noetic}-gazebo-ros-pkgs ros-{melodic|noetic}-gazebo-ros-control
```
