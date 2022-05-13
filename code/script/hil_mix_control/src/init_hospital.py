# Hospital Environment
from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner


import time


##############################
# motion FTS 
ap = ['r0', 'r1', 'r2',
      'r3', 'r4', 'r5',
      'r6','c1', 'c2', 'c3',
      'c4']
loc = [(3.4, 0.00 , 0.00), (-4.00, 5.38, 0.00), (-7.00, 5.38, 0.00),
       (-10.00, 5.38, 0.00), (-10.00, -2.27, 0.00), (-7.00, -2.27, 0.00),
       (-4.00, -2.27, 0.00), (0.00, 0.00, 0.00), (-4.00, 0.00, 0.00),
       (-7.00, 0.00, 0.00), (-10.00, 0.00, 0.00)
         ]
regions = dict()
for k in range(len(ap)):
    regions[loc[k]] = set([ap[k],])
init_pose = loc[0]
robot_motion = MotionFts(regions, set(ap), 'hotel' )
robot_motion.set_initial(list(init_pose))
edges = [(0,7),(7,8),(8,9),
         (9,10),(8,1),(8,6),
         (9,2),(9,5),(10,3),(10,4)]
edge_list = [(loc[e[0]], loc[e[1]]) for e in edges]
robot_motion.add_un_edges(edge_list, unit_cost = 2)


##############################
# action FTS
############# no action model
action = dict()

robot_action = ActionModel(action)

robot_model = [robot_motion, init_pose, robot_action]
##############################
# complete robot model
robot_full_model = MotActModel(robot_motion, robot_action)

##############################
#specify soft and hard tasks

# case one 
# hard_task = '(([]<> (r0 && <> (r8 && <> r7))) && ([]<> (r2 && <> r6)) && ([] !r5))'
# soft_task = '([]! c4)'

# case two
hard_task = '(([]<> r2) && ([]<> r3) && ([]<> r0))'
soft_task = '(([]<> r4) && ([]<> r6))'

sys_model = [robot_full_model, hard_task, soft_task]

