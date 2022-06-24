# Human-in-the-Loop
[![report](https://img.shields.io/badge/Report-pdf-lightgrey)]() [![slides](https://img.shields.io/badge/Slides-ppt-blue)]() 

This work describes the analysis and design choices on building a human-in-the-loop mixed-initiative control under temporal tasks. We analyzed pre-existing techniques through 3D and 2D simulations on various environments, particularly how the presence of a human operator can change to plan edges, both temporarily and permanently.

Our effort was jointly of literary revision and experimentation, whose detailed description can be read in the attached paper. Below a brief peek of the contents: we delve first into previous works conducted in the field to understand its various aspects, following with a preliminary description of the case study framework and the necessary theoretical tools. We subsequently outline the problem formulation as depicted in the reference paper, as we describe the algorithms utilized, step by step and how they are combined together. Finally the simulations conducted, which are contained in this repository, are shown in detail to demonstrate the effectiveness of the illustrated approach, both re-proposing the same environment used in the reference paper and exploiting different solutions designed by us. 

*Contributors:* Edoardo Spinetti, Desiree Bellan, Francesco Vincelli, Alper Calisir


# General Idea
The algorithm proposed in [[1]](#1) implements an online coordination scheme that encapsulates (i) a mixed-initiative continuous controller that ensures all-time safety despite of possible human errors, (ii) a plan adaptation scheme that accommodates new features discovered in the work space and short-term tasks assigned by the operator during run time, and (iii) an iterative inverse reinforcement learning (IRL) algorithm that allows the robot to asymptotically learn the human preference on the parameters during the plan synthesis. 


Taking as inspiration the work done by [[1]](#1), we've relied for our simulations mainly on a 3 dimensional environment supported by a ROS framework. Originally, the algorithm previously presented has been evaluated on two different scenarios, a simulated and a real one, involving two different robotic models: a TIAGO robot and a TURTLEBOT robot. 

Additionally, since the 3D environment suffers from slow processing time and consequently tend to be harder to analyse for more complex algorithm, we've decided to test the implementation also on a costume - based 2D grid environment, completely built in using python3.

In order to evaluate the simulations done, we plotted the trajectories followed by the agents on each cases and compared them to the results obtained by the authors. In particular we want them to :
- Follow the shortest path that satisfy the hard and soft constrains assigned
- Be compliant to the requests of the human operator, changing their plan accordingly, without breaking the hard constraints
- Learn from the human direction, so to reproduce them in the next runs

## Tech Stack
- [ROS]
- [Docker]
- [Gazebo]
- [ROS Noetic] on Ubuntu 20.04 for TURTLEBOT
- [ROS Melodic] on Ubuntu 18.04 for TIAGO

## Documentation
You can read the technical report [here]().

We have used the Test World made by Chao Yao [here](https://github.com/chaolmu/gazebo_models_worlds_collection).

## References
<a id="1">[1]</a> 
Guo, M., Andersson, S. et al..
"Human-in-the-Loop Mixed-Initiative Control Under Temporal Tasks"
2018 IEEE International Conference on Robotics and Automation (ICRA)(2018).



   [ROS]: <http://wiki.ros.org>
   [Docker]: <https://www.docker.com>
   [Gazebo]: <http://gazebosim.org>
   [ROS Noetic]: <http://wiki.ros.org/noetic>
   [ROS Melodic]: <http://wiki.ros.org/melodic>
