#!/usr/bin/env python
# 2D grid for human learning

from re import T
from time import time
from xmlrpc.server import DocXMLRPCRequestHandler
from ltl_tools.ts import MotionMap, distance
from ltl_tools.planner import ltl_planner
from math import sqrt, hypot, exp
from networkx import shortest_path
from hil_mix_control.msg import Task
from tqdm import tqdm
from PIL import Image 

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx

import pickle
import copy
import imageio
import os
import io


show_animation = True


    

class Mapping:
    def __init__(self, ox, oy, resolution, robot_radius):
        """
        Initialize map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.min_x = None
        self.min_y = None
        self.max_x = None
        self.max_y = None
        self.x_width = None
        self.y_width = None
        self.obstacle_map = None
        self.obstacle_pos = []

        self.resolution = resolution
        self.robot_radius = robot_radius
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()
        self.build_graph()
        
    def build_graph(self):
        
        open_set, closed_set, edges = [], [], []
        open_set.append([self.min_x + self.robot_radius, self.min_y + self.robot_radius])
        
        fig = plt.figure(1)
        #extent = fig.get_axes()[0].get_window_extent().transformed(fig.dpi_scale_trans.inverted())
                
        writer = imageio.get_writer('map.gif', mode='I')
        
        while 1:
            search_set = copy.deepcopy(open_set)
            rx, ry = [], []        
            for node in search_set: 
                rx.append(node[0])
                ry.append(node[1])        
                for move_x, move_y,_ in self.motion:
                    poss_node = [node[0] + move_x, node[1] + move_y]
                    if self.verify_node([int(poss_node[0]), int(poss_node[1])]) and poss_node not in closed_set :
                        edges.append([node, poss_node])
                        if poss_node not in open_set:
                            open_set.append(poss_node)
                open_set.remove(node)  
                closed_set.append(node)  
                
            #plt.scatter(rx,ry,color = "lightgrey", marker = 'x')        
            fig.get_axes()[0].scatter(rx,ry,color = "lightgrey", marker = 'x') 
            #fig.savefig('./images/map.png', bbox_inches=extent)
            fig.savefig('./images/map.png')
            image = imageio.imread('./images/map.png')
            writer.append_data(image)
            os.remove('./images/map.png')
            #i += 1
            plt.pause(0.001) 
                     
            if len(open_set) == 0:
                break
        closed_set = [(node[0],node[1]) for node in closed_set]
        edges = [((edge[0][0],edge[0][1]),(edge[1][0],edge[1][1])) for edge in edges]
        self.map = MotionMap(closed_set, edges)
        
        writer.close()
        
    def search(self, position, goal):
        source = min(self.map.nodes, key = lambda x: distance(position,x))
        path = shortest_path(self.map, source = source, target = goal)
        rx = [node[0] for node in path]
        ry = [node[1] for node in path]
        return rx, ry
            
    def verify_node(self, node):
        px = self.calc_position(node[0], self.min_x)
        py = self.calc_position(node[1], self.min_y)

        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False

        if self.obstacle_map[node[0]][node[1]]:
            return False

        return True

    def check_pose(self, pose):
        nearest_obs = min(self.obstacle_pos, key = lambda x: distance(pose[1],x))
        if distance(pose[1],nearest_obs) <= self.robot_radius:
            return False
        else:
            return True
    
    def calc_position(self, id, minp):
        pos = id * self.resolution + minp
        return pos

    def calc_obstacle_pos(self, ox, oy):
        for iox, ioy in zip(ox, oy):
            self.obstacle_pos.append([iox,ioy])   
            
    def calc_obstacle_map(self, ox, oy):
    
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = hypot(iox - x, ioy - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break
        # obstacle positions
        self.calc_obstacle_pos(ox,oy)

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, sqrt(2)],
                  [-1, 1, sqrt(2)],
                  [1, -1, sqrt(2)],
                  [1, 1, sqrt(2)]]
        return motion
    
def norm2(pose1, pose2):
    # 2nd norm distance
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)

def rho(s):
    if (s > 0):
        return exp(-1.0/s)
    else:
        return 0
        
def smooth_mix(tele_control, navi_control, dist_to_trap):
    ds = 5#m
    epsilon = 3#m
    mix_control = [0, 0]
    gain = rho(dist_to_trap-ds)/(rho(dist_to_trap-ds)+rho(epsilon+ds-dist_to_trap))
    mix_control[0] = (1-gain)*navi_control[0] + gain*tele_control[0]
    mix_control[1] = (1-gain)*navi_control[1] + gain*tele_control[1]
    return mix_control, gain

def velocity(vel_xy):
    return sqrt(vel_xy[0]**2 + vel_xy[1]**2)

def onclick(event, model = 'hotel'):
    if event == 'escape':
        global robot_frames, control_frames, planner
        '''
        control_extent = plt.gcf().get_axes()[1].get_window_extent().transformed(plt.gcf().dpi_scale_trans.inverted())
        robot_extent = plt.gcf().get_axes()[0].get_window_extent().transformed(plt.gcf().dpi_scale_trans.inverted())
        '''
        #plt.figure(2).savefig('control.png')
        if len(control_frames) != 0:
            control_frames[-1].save("control.png", format="png")  
            robot_frames[0].save("robot.gif", format="GIF", append_images = robot_frames, save_all=True, duration=100, loop=0)
            control_frames[0].save("control.gif", format="GIF", append_images = control_frames, save_all=True, duration=100, loop=0)  
        #plt.close('all')
        labels = {reg:list(planner.product.graph['ts'].graph['region'].nodes[reg]['label'])[0] 
              for reg in planner.product.graph['ts'].graph['region'].nodes }
        fig3 = plt.figure(3)
        ax3 = plt.subplot(111)
        nx.draw_kamada_kawai(planner.product.graph['ts'].graph['region'], with_labels = True, 
                     labels = labels, node_size = 600, font_size = 10, node_color = '#b3ffcc', arrows = True, ax = ax3)
        fig3.savefig('final_graph.png')
        plt.show()      
        return exit(0)
    elif event == 't':   
        global temp_task    
        taskdata = taskCall(model)             
        s_x = taskdata.sx
        s_y = taskdata.sy
        g_x = taskdata.gx
        g_y = taskdata.gy    
        tsg = taskdata.tsg
        temp_task = (s_x, s_y, g_x, g_y, tsg)
        print ('robot received temporary task <>(%s && <> %s)' %(str((s_x,s_y)), str((g_x,g_y))))
    else:
        global tele_control
        if event == 'up':
            tele_control[1] += 0.1
        elif event == 'right':
            tele_control[0] += 0.1
        elif event == 'down':
            tele_control[1] -= 0.1
        elif event == 'left':
            tele_control[0] -= 0.1
        elif event == 'a':
            tele_control = [0,0]
        
def taskCall(model = 'hotel'):
    keypress = input('Write temporal task in the form : initial_state final_state time').split(' ')
    task = Task()
    try:
        state, goal, tsg = keypress[0], keypress[1], float(keypress[2])
        if model == "hotel":
            ap = ['r0', 'r1', 'r2',
            'r3', 'r4', 'r5',
            'r6', 'r7', 'r8',
            'c1', 'c2', 'c3',
            'c4']
            loc = [(15.00, 15.00), (75.00, 10.00), (110.00, 15.00),
               (165.00, 20.00), (70.00, 45.00), (130.00, 45.00),
               (180.00, 45.00), (25.00, 80.00), (105.00, 80.00),
               (15.00, 50.00), (100.00, 50.00), (160.00, 50.00),
               (70.00, 70.00)]
        elif model == "hospital":
            ap = ['r0', 'r1', 'r2',
                'r3', 'r4', 'r5',
                'r6','c1', 'c2', 'c3',
                'c4']
            loc = [(150.00, 40.00), (75.00, 65.00), (45.00, 65.00),
                (15.00, 65.00), (15.00, 15.00), (45.00, 15.00),
                (75.00, 15.00), (115.00, 40.00), (75.00, 40.00),
                (45.00, 40.00), (15.00, 40.00)]
        if state not in ap or goal not in ap:
                print('State or Goal not in the current model')
        else:
            s = loc[ap.index(state)]
            g = loc[ap.index(goal)]
            task.sx = s[0]; task.sy = s[1]; task.gx = g[0]; task.gy = g[1]; task.tsg = tsg
            
    except Exception as e:
        print('\n Exception {}:'.format(e))        
    return task
    
    
def navVelCall(path, pose):
    global navi_control
    #x1 = next(x for x in path[0] if x != pose[0])
    #y1 = next(x for x in path[1] if x != pose[1])
    x1, y1 = path[0][1], path[1][1]
    x0, y0 = pose[0], pose[1]
    L = distance([x0,y0],[x1,y1])
    dy = (y1 - y0)/L
    dx = (x1 - x0)/L
    navi_control = [dx,dy]
    
def hil_planner(sys_model, initial_beta, robot_name='2D_agent', model_name = 'test', case = 1):
    global robot_pose, navi_control, tele_control, temp_task, robot_frames, control_frames, planner
    robot_full_model, hard_task, soft_task, obstacle = sys_model
    robot_pose = [0, [0, 0]]
    navi_control = [0, 0]
    tele_control = [0, 0]
    mix_control = [0, 0]
    temp_task = None
    ox, oy = obstacle[0], obstacle[1]
    temp_task_s = False
    temp_task_g = False
    flag_task_incop = False
    new_ws = False
    print ('Robot %s: ltl_planner started!' %(robot_name))
    planner = ltl_planner(robot_full_model, hard_task, soft_task, initial_beta, robot_name)
    ####### initial plan synthesis
    planner.optimal()
    print ('Original beta:', initial_beta)
    print ('Initial optimal plan', planner.run.suf_plan)
    #######
    reach_bound = 3.0 # m
    hi_bound = 0.1
    hi_bool = False
    hil = False
    reach_new = False
    #######
    robot_path = []
    reachable_prod_states = set(planner.product.graph['initial'])
    posb_runs = set([(n,) for n in planner.product.graph['initial']])
    #######
    pre_reach_ts = None
    A_robot_pose = []
    A_control = []
    A_beta = []
    controlLists = [[0.0],[0.0],[0.0]]
    # temporal task control
    temp_control = 0
    # REGION SEARCH
    margin = 25#m
    
    labels = {reg:list(planner.product.graph['ts'].graph['region'].nodes[reg]['label'])[0] 
              for reg in planner.product.graph['ts'].graph['region'].nodes }
    nx.draw_kamada_kawai(planner.product.graph['ts'].graph['region'], with_labels = True, 
                     labels = labels, node_size = 600, font_size = 10, node_color = '#b3ffcc', arrows = True)
    plt.show()
    # GIF   
    line, point, temp_text, temp_text_s, temp_text_g = None, None, None, None, None
   
    control_frames, robot_frames = [], []
    
    robot_pose[1] = [list(node) for node in planner.product.graph['ts'].graph['region'].graph['initial']][0]
    pre_reach_ts = None

    # GRID PARAMETERS
    grid_size = 1.0  # [m]
    robot_radius = 3.0  # [m]
    
    # PLOTTING 
    #fig, (ax1, ax2) = plt.subplots(2, 1)
    fig1 = plt.figure(1)
    ax1 = plt.subplot(111)   
    fig2 = plt.figure(2)
    ax2 = plt.subplot(111)
    
    #plt.plot(ox, oy, '.k')
    # plt.grid(True)
    # plt.axis("equal")
    ax1.plot(ox, oy, '.k')
    ax2.grid(True)
    ax1.axis("equal")
    ax1.set_xlim(round(min(ox)) - 10, round(max(ox)) + 10)
       
    #plt.gcf().canvas.mpl_connect('key_press_event', lambda event: onclick(event.key, model_name))
    fig1.canvas.mpl_connect('key_press_event', lambda event: onclick(event.key, model_name))
    fig2.canvas.mpl_connect('key_press_event', lambda event: onclick(event.key, model_name))
    
    mapGraph = Mapping(ox, oy, grid_size, robot_radius)
    
    for node in planner.product.graph['ts'].graph['region'].nodes:
        ax1.add_patch(plt.Circle(node,1,color = 'm', fill = False))

    t0 = time()
    while 1:
        try:
            t = time()-t0
            print ('----------Time: %.2f----------' %t)
            A_robot_pose.append(list(robot_pose))            
            A_control.append([tele_control, navi_control, mix_control, temp_control])
            
            print('robot pose {}'.format(robot_pose))
            
            # check if a new region has been discovered
            if planner.check_region(robot_pose[1], margin):
                print('Visiting known region')
            else:
                print('Visiting new region')
                new_ws = True
                new_region = (round(robot_pose[1][0],0), round(robot_pose[1][1],0))
                planner.update_workspace(['region', new_region])
                
                if planner.check_edge(pre_reach_ts, reach_ts):     
                    print('edge already existing')          
                else:
                    #update workspace
                    print('edge not already existing')
                    print('update workspace')
                    planner.update_workspace(['edge', pre_reach_ts, (new_region,'None')])              
                ax1.add_patch(plt.Circle((new_region[0], new_region[1]), 1, color = 'm', fill = False))
            
            # robot past path update
            reach_ts = planner.reach_ts_node(robot_pose[1], reach_bound)
            if reach_ts == None:
                reach_ts = pre_reach_ts
            
            #----------------------------------    
            if ((reach_ts) and (reach_ts != pre_reach_ts)):
                print ('new region reached', reach_ts)
                robot_path.append(reach_ts)
                    
                # correct errors when robot pose lies in between of two nodes            
                if pre_reach_ts == None:
                    pre_reach_ts = reach_ts

                # check if the visited edge belongs to the workspace
                if planner.check_edge(pre_reach_ts, reach_ts):     
                    print('edge already existing')          
                else:
                    #update workspace
                    print('edge not already existing')
                    print('update workspace')
                    planner.update_workspace(['edge',pre_reach_ts, reach_ts])
                reachable_prod_states = planner.update_reachable(reachable_prod_states, reach_ts)
                posb_runs = planner.update_runs(posb_runs, reach_ts)
                # restart the robot
                # since the number of runs can be extremely long, and consiquently time consuming to compute, they are evaluated
                # any time the region is updated
                if len(posb_runs)>= 5000:
                    posb_runs = set([planner.find_opt_paths_jit(posb_runs),])
                #SendMix(MixPublisher,list(prev_navi_control))              
                reach_new = True
            else:
                reach_new = False
            print(reachable_prod_states)
            #------------------------------
            # mix control inputs
            
            if norm2(tele_control, [0,0]) >= hi_bound:
                print ('--- Human inputs detected ---')
                hi_bool = True
                hil = True
                dist_to_trap = planner.prod_dist_to_trap(robot_pose[1], reachable_prod_states)
                if dist_to_trap >=0:
                    print ('Distance to trap states in product: %.2f' %dist_to_trap)
                    mix_control, gain = smooth_mix(tele_control, navi_control, dist_to_trap)
                    print ('mix_control: %s ||| navi_control: %s ||| tele_control: %s ||| gain: %.2f' %(mix_control, navi_control, tele_control, gain))
                else:
                    print ('No trap states are close')
                    dist_to_trap = 1000
                    mix_control, gain = smooth_mix(tele_control, navi_control, dist_to_trap)
                    print ('mix_control: %s ||| navi_control: %s ||| tele_control: %s ||| gain: %.2f' %(mix_control, navi_control, tele_control, gain))
            else:
                hil = False
                print ('No Human inputs. Autonomous controller used.')
                print('navi_control: %s' %(navi_control))
                mix_control = list(navi_control)
            controlLists[0].append(velocity(navi_control))
            controlLists[1].append(velocity(tele_control))
            controlLists[2].append(velocity(mix_control))
            #print ('robot_path:', robot_path)
            # print 'reachable_prod_states', reachable_prod_states
            # #------------------------------
            # estimate human preference, i.e. beta
            # and update discrete plan
            print('segment %s index %d ' %(planner.segment, planner.index))
            
            if ((reach_new) and (planner.start_suffix())):
                print ('reachable_prod_states', reachable_prod_states)
                prev_navi_control = navi_control
                #SendMix(MixPublisher,[0.0,0.0])
                if hi_bool and not new_ws:
                    print ('------------------------------')
                    print ('---------- In IRL mode now ----------')
                    est_beta_seq, match_score = planner.irl_jit(posb_runs)

                    #est_beta_seq, match_score = planner.irl(robot_path, reachable_prod_states)
                    hi_bool = False
                    A_beta.append(est_beta_seq)
                    print ('------------------------------')
                elif hi_bool and new_ws:
                    planner.optimal(style='ready')
                    opt_suffix = list(planner.run.suffix)
                    planner.set_to_suffix()
                    print ('opt_suffix updated to %s' %str(opt_suffix))
                    print ('-----------------')
                print ('--- New suffix execution---')     
                #SendMix(MixPublisher,list(prev_navi_control))         
                robot_path = [reach_ts]
                reachable_prod_states = planner.intersect_accept(reachable_prod_states, reach_ts)
                posb_runs = set([(n,) for n in reachable_prod_states])
            #------------------------------
            # satisfy temporary task
            if temp_task:
                if not flag_task_incop:
                    if temp_text != None:
                        temp_text.remove()
                    if temp_text_s != None:
                        temp_text_s.remove()
                    if temp_text_g != None:
                        temp_text_g.remove()
                    reg_s = (temp_task[0], temp_task[1])
                    reg_g = (temp_task[2], temp_task[3])
                    if planner.check_node(reg_s) and planner.check_node(reg_g):
                        planner.add_temp_task(temp_task)
                        flag_task_incop = True
                        temp_text = ax1.text(0, 117,'Temporary Task: from ({},{}) to ({},{}) in {} sec'.format(temp_task[0], temp_task[1],temp_task[2], temp_task[3], temp_task[4]))
                        t_temp = time() - t0
                    else: 
                        temp_task = None
                        temp_text = ax1.text(0, 117,r"Task <>({} && <> {}) is unfeasible""\n"r"cannot be executed".format(reg_s, reg_g))
                            
                if ((reach_ts) and (reach_ts[0] == reg_s) and (pre_reach_ts[0] != reg_s)):
                    temp_task_s = True
                    time_s = t - t_temp
                    temp_control = 1
                    #print ('robot reaches pi_s in the temp task:{} in {} seconds'.format(reg_s, time_s))
                    temp_text_s = ax1.text(0, 110,'Reaches {} in {:.2f} seconds'.format(reg_s, time_s))
                if (temp_task_s) and ((reach_ts) and (reach_ts[0] == reg_g)):
                    temp_task_g = True
                    time_g = t - t_temp
                    #print ('robot reaches pi_g in the temp task:{} in {} seconds'.format(reg_g, time_g))
                    temp_text_g = ax1.text(0, 100,'Reaches {} in {:.2f} seconds'.format(reg_g, time_g))
                if (temp_task_s) and (temp_task_g):
                    print ('robot accomplished temporary task <>({} && <> {}) in {} seconds'.format(reg_s, reg_g, time_g-time_s))
                    temp_task = None
                    temp_task_s = False
                    temp_task_g = False
                    flag_task_incop = False
                    temp_control = 0

            #------------------------------
            # # plan execution
            current_goal = planner.next_move      
            pre_reach_ts = reach_ts
            # next move is action
            if isinstance(current_goal, str):
                print ('the robot next_move is an action, currently not implemented for %s' %robot_name)
                break
            # next move is motion
            
            if ((reach_ts) and (reach_ts[0] == current_goal)):    
                print('Goal %s reached by %s.' %(str(current_goal),str(robot_name)))
                planner.find_next_move()
            else: 
                if line:
                    line.remove()
                if point:
                    point.remove()
                s = robot_pose[1]  
                g = list(current_goal)                        
                rx, ry = mapGraph.search(((s[0]), s[1]), (g[0], g[1]))
                line, = ax1.plot(rx, ry, ':k')
                
                # update robot figure
                plt.figure(1)
                
                if hil:
                    point, = ax1.plot(robot_pose[1][0], robot_pose[1][1], "or") 
                elif temp_control == 1:
                    point, = ax1.plot(robot_pose[1][0], robot_pose[1][1], "og")
                else:
                    point, = ax1.plot(robot_pose[1][0], robot_pose[1][1], "ob")
                               
                navVelCall([rx,ry],s)
                robot_pose[1][0] += mix_control[0]
                robot_pose[1][1] += mix_control[1]
                if not mapGraph.check_pose(robot_pose):
                    robot_pose[1][0] -= mix_control[0]
                    robot_pose[1][1] -= mix_control[1]
                
                print('Goal %s sent to %s.' %(str(current_goal),str(robot_name)))
                              
                text_nav = ax1.text(-20,130,'navi. control = ({:.2f},{:.2f})'.format(navi_control[0],navi_control[1]))
                text_mix = ax1.text(70, 130, 'mix control: ({:.2f},{:.2f})'.format(mix_control[0], mix_control[1]))
                text_tele = ax1.text(150, 130, 'hil control: ({:.2f},{:.2f})'.format(tele_control[0], tele_control[1]))
                text_beta = ax1.text(80, -20, 'BETA: {:.2f}'.format(planner.beta))
                
                plt.pause(0.001)    
                
                robot_frames.append(Image.fromarray(np.array(fig1.canvas.renderer.buffer_rgba())))
                
                text_nav.remove()
                text_mix.remove()
                text_tele.remove()
                text_beta.remove()
                                            
                # update control figure  
                plt.figure(2)
                  
                nav, = ax2.plot(controlLists[0], linestyle='--',linewidth=2.0, color='blue',label=r'$u_r[v]$',zorder = 3)
                tele, = ax2.plot(controlLists[1], linestyle='--',linewidth=2.0, color='red',label=r'$u_h[v]$',zorder = 4)
                mix, = ax2.plot(controlLists[2], linestyle='-',linewidth=2.0, color='black',label=r'$u[v]$',zorder = 2)
                
                ax2.legend(ncol=3,loc='upper left',borderpad=0.1, labelspacing=0.2, columnspacing= 0.5)
                
                plt.pause(0.001)
                
                control_frames.append(Image.fromarray(np.array(fig2.canvas.renderer.buffer_rgba()))) 
                
                nav.remove()
                tele.remove()
                mix.remove()    
                                     
        except KeyboardInterrupt:
            pickle.dump([A_robot_pose, A_control, A_beta], open('/home/desiree/catkin_ws/src/mix_initiative/hil_mix_control/src/data/turtlebot_{}.p'.format(model_name), 'wb'))
            print ('result saved')
            plt.show() 
            break
            #print (A_robot_pose)
    pickle.dump([A_robot_pose, A_control, A_beta], open('/home/desiree/catkin_ws/src/mix_initiative/hil_mix_control/src/data/turtlebot_{}.p'.format(model_name), 'wb'))
    print ('result saved')
    #print (A_robot_pose)

        

    
if __name__ == "__main__":
    from argparse import ArgumentParser
    
    parser = ArgumentParser(description='hil mix planner turtlebot')
    parser.add_argument('--model','-m', default = 'test', type = str,
                    help='Model : [test, hospital, hotel]')
    parser.add_argument('--beta','-b', default = 0, type = float,
                    help='Initial beta value')
    parser.add_argument('--case','-c', default = 1, type = int,
                        help = 'task case [1,2], 1 for delivery, 2 for surveillance.')
    args = parser.parse_args()
    model = args.model
    beta = args.beta
    case = args.case
    
    from init_2d import sys_model
    system_model = sys_model(case = case, model = model)
    
    hil_planner(system_model, beta, model_name = model, case = case)
