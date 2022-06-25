# EXPERIMENTS
List of the experiments done 
# Ros Turtlebot
1. Hotel Environment
   1. Case 1: Delivery Constraints
      - Without Human in the Loop
        - ![#f03c15](https://via.placeholder.com/15/f03c15/f03c15.png) Starting beta = 30 and feasible soft constraint
        - Starting beta = 30 and unfeasible soft constraint [time t0](ROS_Turtlebot/Hotel/Delivery/NO_HIL/t0_1.png), [time t](ROS_Turtlebot/Hotel/Delivery/NO_HIL/t_1.png), [control plot](ROS_Turtlebot/Hotel/Delivery/NO_HIL/control_1.png))
        - ![#f03c15](https://via.placeholder.com/15/f03c15/f03c15.png) Starting beta = 0
      - With Human in the Loop 
        - Starting beta = 30, after IRL beta decrease (pass though c4)
        - Starting beta = 30, afterl IRL beta doesn't change (pass though r5)
        - Temporal task: from r0 to r7 in 1000 sec (feasible)
        - Temporal task: from r0 to r7 in 10 sec (feasible with delay)
        - Temporal task: from r8 to r5 in 50 sec (not feasible)
        - Modify smooth mix parameters: ds = 5 m and epsilon = 2 m 
   2. Case 2: Surveillance Contraints ([buchi automata hard constraints](ROS_Turtlebot/Hotel/Surveillance/buchi_hard.gif) and [soft constraints](ROS_Turtlebot/Hotel/Surveillance/buchi_soft.gif))
      
      hard_task = $ (\square\lozenge r2) \wedge (\square\lozenge r3) \wedge (\square\lozenge r8)$ 
      
      soft_task = $\square(c3 \implies (\lozenge r6))$
      
      - Without Human in the Loop
        - ![#f03c15](https://via.placeholder.com/15/f03c15/f03c15.png) Starting beta = 30
        - ![#f03c15](https://via.placeholder.com/15/f03c15/f03c15.png) Starting beta = 0
        - Incomplete workspace: missing edge 
        - Incomplete workspace: missing node
      - With Human in the Loop 
        - Starting beta = 0, after IRL beta increase (pass though r4) ([time t0](ROS_Turtlebot/Hotel/Surveillance/HIL/t0_1.png), [time t](ROS_Turtlebot/Hotel/Surveillance/HIL/t_1.png), [beta plot]() and [control plot]())
        - Temporal task: from r0 to r7 in 1000 sec (feasible)
        - Temporal task: from r0 to r7 in 10 sec (feasible with delay)
        - Temporal task: from r0 to r4 in 50 sec (feasible, does not change beta)
2. Hospital Environment
   1. Case 1: Delivery Constraints ([buchi automata](ROS_Turtlebot/Hospital/Delivery/buchi_hard.gif))
   
      hard constraints = $\square\lozenge (r0 \wedge \lozenge (r2 \wedge \lozenge r4) \wedge (\square \lnot r3)) $ 
      
      soft constraints = $\square\lnot r6$
      
      - ![#f03c15](https://via.placeholder.com/15/f03c15/f03c15.png) Without Human in the Loop 
        - Starting beta = 0 ([time t0](ROS_Turtlebot/Hospital/Delivery/NO_HIL/t0_1.png), [time_t](ROS_Turtlebot/Hospital/Delivery/NO_HIL/t_1.png), [control plot](ROS_Turtlebot/Hospital/Delivery/NO_HIL/control_1.png))
        - Starting beta = 30 ([time t0](ROS_Turtlebot/Hospital/Delivery/NO_HIL/t0_2.png), [time_t](ROS_Turtlebot/Hospital/Delivery/NO_HIL/t_2.png), [control plot](ROS_Turtlebot/Hospital/Delivery/NO_HIL/control_2.png))
      - With Human in the Loop
        - ![#f03c15](https://via.placeholder.com/15/f03c15/f03c15.png) Starting beta = 30, after IRL beta decrease (pass though r6) ([time t0](ROS_Turtlebot/Hospital/Delivery/HIL/t0_1.png), [time_t](ROS_Turtlebot/Hospital/Delivery/HIL/t_1.png), [control plot](ROS_Turtlebot/Hospital/Delivery/HIL/control_1.png) and [beta plot](ROS_Turtlebot/Hospital/Delivery/HIL/beta_1.png))
        - Temporal task: from r2 to r5 in 500 sec (feasible)
        - Temporal task: from r2 to r5 in 10 sec (feasible, delay)
        - Temporal task: from r5 to r3 in 500 sec (not feasible)
        - Temporal task: from r5 to r6 in 500 sec (feasible, does not change beta)
   2. Case 2: Surveillance Constraints ([buchi automata](ROS_Turtlebot/Hospital/Surveillance/buchi_hard.gif))
      
      hard_task = $(\square\lozenge c2) \wedge (\square \lozenge c3) \wedge (\square\lozenge c4)$
      
      soft_task = $ \square (c3 \implies (\lnot c4 U \lozenge r2))$ 
      - ![#f03c15](https://via.placeholder.com/15/f03c15/f03c15.png) Without Human in the Loop 
        -  Starting beta = 30 ([time t0](ROS_Turtlebot/Hospital/Surveillance/NO_HIL/t0_1.png), [time t](ROS_Turtlebot/Hospital/Surveillance/NO_HIL/t_1.png), [control plot](ROS_Turtlebot/Hospital/Surveillance/NO_HIL/control_1.png)
        - Starting beta = 0 ([time t0](ROS_Turtlebot/Hospital/Surveillance/NO_HIL/t0_2.png), [time t](ROS_Turtlebot/Hospital/Surveillance/NO_HIL/t_2.png), [control plot](ROS_Turtlebot/Hospital/Surveillance/NO_HIL/control_2.png)
        
      - With Human in the Loop
         - Unknown environment (node, doesn't affect plan)
         - Unknown environment (edge, doesn't affect plan)
         - Unknown environment (edge, does affect plan)
# 2D Grid 
1. Hotel Environment ([grid search](2D%20Grid/Hotel/map.gif))
   1. Case 1: Delivery Constraints
      - ![#f03c15](https://via.placeholder.com/15/f03c15/f03c15.png) Without Human in the Loop
        - Starting beta = 310 (the constraint affect significantly the run cost so beta needs to be really high) ([motion animation](2D%20Grid/Hotel/Delivery/NO_HIL/robot_2.gif) and [graph animation](2D%20Grid/Hotel/Delivery/NO_HIL/control_2.gif))
        -  Starting beta = 0 ([motion animation](2D%20Grid/Hotel/Delivery/NO_HIL/robot_1.gif) and [graph animation](2D%20Grid/Hotel/Delivery/NO_HIL/control_1.gif))
      - With Human in the Loop 
        - Starting beta = 30, after IRL beta decrease (pass though c4)
        - Starting beta = 30, afterl IRL beta doesn't change (pass though r5)
        - Temporal task: from r0 to r7 in 1000 sec (feasible)
        - Temporal task: from r0 to r7 in 10 sec (feasible with delay)
        - Temporal task: from r8 to r5 in 50 sec (not feasible)
        - Modify smooth mix parameters: ds = 5 m and epsilon = 2 m 
   2. Case 2: Surveillance Contraints
      - Without Human in the Loop
        - Starting beta = 30
        - ![#f03c15](https://via.placeholder.com/15/f03c15/f03c15.png) Starting beta = 0 ([motion animation](2D%20Grid/Hotel/Surveillance/NO_HIL/robot_1.gif) and [graph animation](2D%20Grid/Hotel/Surveillance/NO_HIL/control_1.gif))
        - Incomplete workspace: missing edge 
        - Incomplete workspace: missing node
      - With Human in the Loop 
        - Starting beta = 0, after IRL beta increase (pass though r4)
        - Temporal task: from r0 to r7 in 1000 sec (feasible)
        - Temporal task: from r0 to r7 in 10 sec (feasible with delay)
        - Temporal task: from r0 to r4 in 50 sec (feasible, does not change beta)
2. Hospital Environment ([grid search](2D%20Grid/Hospital/map.gif))
   1. Case 1: Delivery Constraints
      - ![#f03c15](https://via.placeholder.com/15/f03c15/f03c15.png) Without Human in the Loop
        - Starting beta = 0 ([motion animation](2D%20Grid/Hospital/Delivery/NO_HIL/robot_1.gif) and [graph animation](2D%20Grid/Hospital/Delivery/NO_HIL/control_1.gif))
        - Starting beta = 30 ([motion animation](2D%20Grid/Hospital/Delivery/NO_HIL/robot_2.gif) and [graph animation](2D%20Grid/Hospital/Delivery/NO_HIL/control_2.gif))
      - With Human in the Loop
        - Starting beta = 30, after IRL beta decrease (pass though r6)
        - Temporal task: from r2 to r5 in 500 sec (feasible)
        - Temporal task: from r2 to r5 in 10 sec (feasible, delay)
        - Temporal task: from r5 to r3 in 500 sec (not feasible)
        - Temporal task: from r5 to r6 in 500 sec (feasible, does not change beta)
   2. Case 2: Surveillance Constraints 
      - Without Human in the Loop
        - ![#f03c15](https://via.placeholder.com/15/f03c15/f03c15.png) Starting beta = 0 ([motion animation](2D%20Grid/Hospital/Surveillance/NO_HIL/robot_1.gif) and [graph animation](2D%20Grid/Hospital/Surveillance/NO_HIL/control_1.gif))
        - ![#f03c15](https://via.placeholder.com/15/f03c15/f03c15.png) Starting beta = 30 (\*) ([motion animation](2D%20Grid/Hospital/Surveillance/NO_HIL/robot_3.gif) and [graph animation](2D%20Grid/Hospital/Surveillance/NO_HIL/control_3.gif))
      - With Human in the Loop
         - Unknown environment (node, doesn't affect plan)
         - Unknown environment (edge, doesn't affect plan)
         - Unknown environment (edge, does affect plan)

(*) hard constraints = $\square\lozenge c2 \wedge \square\lozenge c3 \wedge \square\lozenge c4 $ soft constraints = $\square\lozenge r2$
