from re import I
from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from math import sqrt, atan2, cos, sin

##############################
# # motion FTS 
ox, oy = [], []

def line(v1, v2):
    x1, y1 = v1[0], v1[1]
    x2, y2 = v2[0], v2[1]
    n = round(sqrt((x1 - x2)**2 + (y1 - y2)**2))   
    theta = atan2((y2 - y1),(x2 - x1))
    for i in range(n):
        ox.append(x1)
        oy.append(y1)
        x1 += cos(theta)
        y1 += sin(theta)
 
def sys_model(case = 1, model = 'test') :
    
    ##############################
    # build grid model
    if model == 'test':
        # set known nodes and edges
        ap = ['s', 'g']
        loc = [(-6.00, -6.00), (50.00, 50.00)]
        edges = [(0,1)]
        # set obstacle positions   
        line([-10,-10], [60,-10])    
        line([60,-10],[60,60])
        line([-10,60],[61,60])
        line([-10,-10],[-10,61])
        line([20,-10],[20,40])
        line([40,20],[40,60])
        #specify soft and hard tasks
        if case == 1:
            hard_task = '([]<> s && []<> g)'
            soft_task = None
    elif model == 'hospital':
        # set known nodes and edges
        ap = ['r0', 'r1', 'r2',
        'r3', 'r4', 'r5',
        'r6','c1', 'c2', 'c3',
        'c4']
        loc = [(150.00, 40.00), (75.00, 65.00), (45.00, 65.00),
               (15.00, 65.00), (15.00, 15.00), (45.00, 15.00),
               (75.00, 15.00), (115.00, 40.00), (75.00, 40.00),
               (45.00, 40.00), (15.00, 40.00)]
        edges = [(0,7),(7,8),(8,9),
          (9,10),(8,1),(8,6),
          (9,2),(9,5),(10,3),(10,4)]
        # set obstacle positions
        line([0,0],[160,0])
        line([0,80],[160,80])
        line([0,0],[0,80])
        line([160,0],[160,80])
        line([140,0],[140,10])
        line([140,20],[140,60])
        line([140,70],[140,80])
        line([90,0],[90,30])
        line([90,50],[90,80])
        line([60,0],[60,30])
        line([60,50],[60,80])
        line([30,0],[30,30])
        line([30,50],[30,80])
        line([0,30],[10,30])
        line([20,30],[40,30])
        line([50,30],[70,30])
        line([80,30],[90,30])
        line([0,50],[10,50])
        line([20,50],[40,50])
        line([50,50],[70,50])
        line([80,50],[90,50])
        #specify soft and hard tasks
        if case == 1:
            # case one : delivery
            hard_task = '([]<> (r0 && <> (r2 && <> r4)) && ([] !r3))'
            soft_task = '([] !r6)'
        elif case == 2: 
            # case two : surveillance
            hard_task = '(([]<> c2) && ([]<>c3) && ([]<>c4))'
            #soft_task = '([]<> (c3 -> (X r2)))'  
            soft_task = '([]<>r2)'  
    elif model == 'hotel':
        # set known nodes and edges
        ap = ['r0', 'r1', 'r2',
            'r3', 'r4', 'r5',
            'r6', 'r7', 'r8',
            'c1',  'c3',
            'c4']
        loc = [(15.00, 15.00), (75.00, 10.00), (110.00, 15.00),
               (165.00, 20.00), (70.00, 45.00), (130.00, 45.00),
               (180.00, 45.00), (25.00, 80.00), (105.00, 80.00),
               (15.00, 50.00), (160.00, 50.00),
               (70.00, 70.00)]
        edges = [(0,2), (0,4), (0,9), (1,4), (1,2), 
          (2,4), (3,10), (5,8), (5,10), (5,10),
          (6,10),(7,9),(7,11),(8,11),(8,10),
          (9,11),(10,11)]
        # set obstacles positions
        line([0,0],[190,0])
        line([0,90],[190,90])
        line([0,0],[0,90])
        line([190,0],[190,90])
        line([30,0],[30,20])
        line([30,30],[30,60])
        line([50,30],[50,60])
        line([90,30],[90,60])
        line([110,20],[110,60])
        line([150,30],[150,60])
        line([50,0],[50,20])
        line([50,80],[50,90])
        line([90,0],[90,10])
        line([90,80],[90,90])
        line([110,0],[110,10])
        line([130,0],[130,30])
        line([130,80],[130,90])
        line([150,0],[150,20])
        line([150,80], [150,90])   
        line([170,20],[170,40])
        line([170,50],[170,60])
        line([0,80],[20,80])
        line([30,80],[60,80])
        line([70,80],[100,80])
        line([110,80],[130,80])
        line([150,80], [160,80])
        line([170,80], [190, 80])
        line([30,60],[40,60])
        line([50,60],[90,60])
        line([110,60],[130,60])
        line([140,60],[150,60])
        line([170,60],[190,60])
        line([0,30],[10,30])
        line([20,30],[30,30])
        line([30,40],[50,40])
        line([50,10],[70,10])
        line([80,10],[90,10])
        line([110,30],[150,30])
        line([150,20],[160,20])
        line([170,20],[190,20])
        line([50,30],[70,30])
        line([80,30],[90,30])
        #specify soft and hard tasks
        # case one : delivery
        if case == 1:
            hard_task = '(([]<> (r0 && <> (r8 && <> r7))) && ([]<> (r2 && <> (r3 || r6))) && ([] !r5))'
            soft_task = '(([] !c4))'
        elif case == 2:
            # case two : surveillance
            hard_task = '(([]<> r2) && ([]<> r3) && ([]<> r8))'
            soft_task = '([]<> r4)'
        
    #############################
    regions = dict()
    for k in range(len(ap)):
        regions[loc[k]] = set([ap[k],])
    init_pose = loc[0]
    robot_motion = MotionFts(regions, set(ap), model )
    robot_motion.set_initial(list(init_pose))
    edge_list = [(loc[e[0]], loc[e[1]]) for e in edges]
    robot_motion.add_un_edges(edge_list, unit_cost = 2)
    #############################
    # # action FTS
    # ############# no action model
    action = dict()
    robot_action = ActionModel(action)

    robot_model = [robot_motion, init_pose, robot_action]
    ##############################
    # complete robot model
    robot_full_model = MotActModel(robot_motion, robot_action)


    return  [robot_full_model, hard_task, soft_task, [ox,oy]]