from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner


import time


##############################
# motion FTS
ap = ['r0', 'r1', 'r2', 'r3']
loc = [(-0.14, 0.28, -1.6), (-0.02, 1.5, 1.47), (-1.8, -0.46, 3.06), (0.39, -1.8, -1.03)]
regions = {   loc[0]: set(['r0',]),
              loc[1]: set(['r1',]),
              loc[2]: set(['r2',]),
              loc[3]: set(['r3',]),              
}

init_pose = loc[0]
robot_motion = MotionFts(regions, set(ap), 'office' )
robot_motion.set_initial(list(init_pose))
edge_list = [(loc[0],loc[1]), (loc[0],loc[2]), (loc[3],loc[0])]
robot_motion.add_un_edges(edge_list, unit_cost = 0.1)


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

# hard_task = '(([]<> r0) && ([]<> r1))'
# soft_task = '(([]<> r2) && ([]<> r3))'
hard_task = '(([]<> r0) && ([]<> r1))'
soft_task = '(([]<> r2) && ([]<> r3))'

# hard_task = '(([]<> r3) && ([]<> r1))'
# soft_task = '[] ! r0'

sys_model = [robot_full_model, hard_task, soft_task]

