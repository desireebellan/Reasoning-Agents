# EXPERIMENTS
List of the experiments done 
# Ros Turtlebot
1. Hotel Environment
   1. Case 1: Delivery Constraints
      - Without Human in the Loop
        - Starting beta = 30
        - Starting beta = 0
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
        - Starting beta = 0
        - Incomplete workspace: missing edge 
        - Incomplete workspace: missing node
      - With Human in the Loop 
        - Starting beta = 0, after IRL beta increase (pass though r4)
        - Temporal task: from r0 to r7 in 1000 sec (feasible)
        - Temporal task: from r0 to r7 in 10 sec (feasible with delay)
        - Temporal task: from r0 to r4 in 50 sec (feasible, does not change beta)
2. Hospital Environment
   1. Case 1: Delivery Constraints
      - With Human in the Loop
        - Starting beta = 0
        - Starting beta = 30
      - Without Human in the Loop
        - Starting beta = 30, after IRL beta decrease (pass though r6)
        - Temporal task: from r2 to r5 in 500 sec (feasible)
        - Temporal task: from r2 to r5 in 10 sec (feasible, delay)
        - Temporal task: from r5 to r3 in 500 sec (not feasible)
        - Temporal task: from r5 to r6 in 500 sec (feasible, does not change beta)
   2. Case 2: Surveillance Constraints 
      - With Human in the Loop
        - Starting beta = 0
        - Starting beta = 30
      - Without Human in the Loop
         - Unknown environment (node, doesn't affect plan)
         - Unknown environment (edge, doesn't affect plan)
         - Unknown environment (edge, does affect plan)
# 2D Grid 
1. Hotel Environment
   1. Case 1: Delivery Constraints
      - Without Human in the Loop
        - Starting beta = 30
        - Starting beta = 0
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
        - Starting beta = 0
        - Incomplete workspace: missing edge 
        - Incomplete workspace: missing node
      - With Human in the Loop 
        - Starting beta = 0, after IRL beta increase (pass though r4)
        - Temporal task: from r0 to r7 in 1000 sec (feasible)
        - Temporal task: from r0 to r7 in 10 sec (feasible with delay)
        - Temporal task: from r0 to r4 in 50 sec (feasible, does not change beta)
2. Hospital Environment
   1. Case 1: Delivery Constraints
      - With Human in the Loop
        - Starting beta = 0
        - Starting beta = 30
      - Without Human in the Loop
        - Starting beta = 30, after IRL beta decrease (pass though r6)
        - Temporal task: from r2 to r5 in 500 sec (feasible)
        - Temporal task: from r2 to r5 in 10 sec (feasible, delay)
        - Temporal task: from r5 to r3 in 500 sec (not feasible)
        - Temporal task: from r5 to r6 in 500 sec (feasible, does not change beta)
   2. Case 2: Surveillance Constraints 
      - With Human in the Loop
        - Starting beta = 0
        - Starting beta = 30
      - Without Human in the Loop
         - Unknown environment (node, doesn't affect plan)
         - Unknown environment (edge, doesn't affect plan)
         - Unknown environment (edge, does affect plan)