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

utility/mix_u/mix_u.py

```
def smooth_mix(tele_control, navi_control, dist_to_trap):
    ds = 0.4
    epsilon = 0.1
    mix_control = 0
    gain = rho(dist_to_trap-ds)/(rho(dist_to_trap-ds)+rho(epsilon+ds-dist_to_trap))
    mix_control = navi_control + gain*tele_control
    return mix_control, gain
```
hil_mix_control/src/hil_mix_planner_turtlebot.py

```
def smooth_mix(tele_control, navi_control, dist_to_trap):
    ds = 0.3
    epsilon = 0.1
    mix_control = [0, 0]
    gain = rho(dist_to_trap-ds)/(rho(dist_to_trap-ds)+rho(epsilon+ds-dist_to_trap))
    mix_control[0] = (1-gain)*navi_control[0] + gain*tele_control[0]
    mix_control[1] = (1-gain)*navi_control[1] + gain*tele_control[1]
    return mix_control, gain
```
hil_mix_control/src/hil_mix_planner_tiago.py

```
def smooth_mix(tele_control, navi_control, dist_to_trap):
    if tele_control[0] > 0.4:
        tele_control[0] = 0.4
    ds = 2.5
    epsilon = 0.5
    mix_control = [0, 0]
    gain = rho(dist_to_trap-ds)/(rho(dist_to_trap-ds)+rho(epsilon+ds-dist_to_trap))
    mix_control[0] = (1-gain)*navi_control[0] + gain*tele_control[0]
    mix_control[1] = (1-gain)*navi_control[1] + gain*tele_control[1]
    return mix_control, gain
```
Robot informations and main function are hold inside the class ltl_planner:

```
class ltl_planner(object):
	def __init__(self, ts, hard_spec, soft_spec, beta)

```
```
The optimal initial path is computed through a nested Dijkstra's search:
def dijkstra_plan_networkX(product, beta=10):
	...
	for prod_target in product.graph['accept']:
                if prod_target in product.predecessors(prod_target):
                        loop[prod_target] = (product.edge[prod_target][prod_target]["weight"], [prod_target, prod_target])
                        continue
                else:
                        cycle = {}
                        loop_pre, loop_dist = dijkstra_predecessor_and_distance(product, prod_target)
                        for target_pred in product.predecessors_iter(prod_target):
                                if target_pred in loop_dist:
                                        cycle[target_pred] = product.edge[target_pred][prod_target]["weight"] + loop_dist[target_pred]
                        if cycle:
                                opti_pred = min(cycle, key = cycle.get)
                                suffix = compute_path_from_pre(loop_pre, opti_pred)
                                loop[prod_target] = (cycle[opti_pred], suffix)
	# shortest line
	for prod_init in product.graph['initial']:
                line = {}
		line_pre, line_dist = dijkstra_predecessor_and_distance(product, prod_init)
		for target in loop.iterkeys():
			if target in line_dist:
				line[target] = line_dist[target]+beta*loop[target][0]
		if line:
			opti_targ = min(line, key = line.get)
			prefix = compute_path_from_pre(line_pre, opti_targ)
			precost = line_dist[opti_targ]
			runs[(prod_init, opti_targ)] = (prefix, precost, loop[opti_targ][1], loop[opti_targ][0])
	# best combination
	if runs:
		prefix, precost, suffix, sufcost = min(runs.values(), key = lambda p: p[1] + beta*p[3])
		run = ProdAut_Run(product, prefix, precost, suffix, sufcost, precost+beta*sufcost)
		....
```

## Simulations

## Results
*graphs, gifs, photos...*
## References
<a id="1">[1]</a> 
Guo, M., Andersson, S. et al..
"Human-in-the-Loop Mixed-Initiative Control Under Temporal Tasks"
2018 IEEE International Conference on Robotics and Automation (ICRA)(2018).
## Instructions
*Here will go the intructions to compile and execute the various simulations*
### TURTLEBOT simulations
* Install ROS [here](http://wiki.ros.org/ROS/Installation)(choose the release that best fits your distro)
* Clone the turtlebot3, turtlebot3_msg and turtlebot3_simulations packages inside ~/catking_ws/src
```
~/catkin_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git -b -{melodic|noetic}-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b -{melodic|noetic}-devel
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ..
catkin_make
```
### Solved Issues
* If turtlebot3_teleop does not work with gazebo, try to reinstall gazebo_ros_pkgs
```
sudo apt-get install ros-{melodic|noetic}-gazebo-ros-pkgs ros-{melodic|noetic}-gazebo-ros-control
```
