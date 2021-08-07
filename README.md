# Human in the loop
This repository consitutes the final project for the course "Reasoning Agents" (Master course Artificial Intelligence and Robotics, Sapienza University, Rome) held by prof. Giuseppe De Giacomo during the accademic year 2020/2021. 
## Introduction

## Preliminary Theory
## Problem formulation
[[1]](#1)
## Simulations
*Here will go the explaination of the most important part of the code, what we took from pre existing works and what we implemented*
```
def smooth_mix(tele_control, navi_control, dist_to_trap):
    ds = 0.4
    epsilon = 0.1
    mix_control = 0
    gain = rho(dist_to_trap-ds)/(rho(dist_to_trap-ds)+rho(epsilon+ds-dist_to_trap))
    mix_control = navi_control + gain*tele_control
    #mix_control = (1-gain)*navi_control + gain*tele_control
    return mix_control, gain
```
## Results
*graphs, gifs, photos...*
## References
<a id="1">[1]</a> 
Guo, M., Andersson, S. et al..
"Human-in-the-Loop Mixed-Initiative Control Under Temporal Tasks"
2018 IEEE International Conference on Robotics and Automation (ICRA)(2018).
## Instructions
*Here will go the intructions to compile and execute the various simulations*
