# TURTLEBOT3 SIMULATIONS
Here you can find the instructions to succesfully run the experiments using Turtlebot3.

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
wget ""
```
* Inside the directory ~/catkin_ws source the environment defined in the previous step
```
cd ~/catkin_ws
source ~/.bashrc
```
* Run the following instructions on different terminals
```
roslaunch mix_initiative_turtlebot turtlebot3_hotel.launch
```

## INSTRUCTION
* install packages
* modify packages 
* run roslaunch experiment
* run roslaunch teleop
* run rosrun
## SOLVED ISSUES
