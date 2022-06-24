# TURTLEBOT3 SIMULATIONS
Here you can find the instructions to succesfully run the experiments using Turtlebot3.

## INSTALL ROS
## DOWNLOAD PACKAGES
* Clone the turtlebot3 and turtlebot3_msg packages inside ~/catkin_ws/src
```
cd ~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b -{melodic|noetic}-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b -{melodic|noetic}-devel
```
* Modify the source file to use the BURGER model:

Open the file from terminal
```
gedit ~/.bashrc
```
Copy paste the following instructions:
```
export TURTLEBOT3_MODEL=burger
```
* Download the folders script and turtlebot3_experiments inside catkin_ws
```
cd ~/catkin_ws/src
wget "https://downgit.github.io/#/home?url=https://github.com/desireebellan/Reasoning-Agents/tree/main/code/script"
wget "https://downgit.github.io/#/home?url=https://github.com/desireebellan/Reasoning-Agents/tree/main/code/turtlebot3_experiments"
```

## MODIFY PACKAGES
## BUILD
## RUN
* Run the following instructions on different terminals
```
roslaunch turtlebot3_files turtlebot3_{hospital|hotel}.launch
roslaunch turtlebot3_files temp_task.launch model:="{hospital|hotel}"
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
* Run the following instruction on a different terminal (the parameters can be changed to execute different experiments)
```
rosrun hil_mix_control hil_mix_planner_turtlebot.py -b 30 -c 1 -m hotel
```

## INSTRUCTION
* install packages
* modify packages 
* run roslaunch experiment
* run roslaunch teleop
* run rosrun
## SOLVED ISSUES
### Solved Issues
* If turtlebot3_teleop does not work with gazebo, try to reinstall gazebo_ros_pkgs
```
sudo apt-get install ros-{melodic|noetic}-gazebo-ros-pkgs ros-{melodic|noetic}-gazebo-ros-control
```

