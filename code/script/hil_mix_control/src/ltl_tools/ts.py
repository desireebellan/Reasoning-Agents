# -*- coding: utf-8 -*-

from ltl_tools.boolean_formulas.parser import parse as parse_guard

from math import sqrt
from networkx.classes.digraph import DiGraph
import networkx as nx



def distance(pose1, pose2):
    return (sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001)

def reach_waypoint(pose, waypoint, margin):
    if distance(pose, waypoint)<=margin:
        return True
    else:
        return False
    
class MotionMap(DiGraph):
    def __init__(self, nodes, edges):
        DiGraph.__init__(self, initial=set())
        for n in nodes:
            self.add_node(n, status='confirmed')
        self.add_un_edges(edges)
        
        print('Motion map constructed with %d states and %s transitions' %(len(self.nodes()), len(self.edges())))
            
    def add_un_edges(self, edge_list, unit_cost=1):
        for edge in edge_list:
            f_node = edge[0]
            t_node = edge[1]
            dist = distance(f_node, t_node)
            self.add_edge(f_node, t_node, weight=dist*unit_cost)
            self.add_edge(t_node, f_node, weight=dist*unit_cost)
        for node in list(self.nodes()):
            self.add_edge(node, node, weight=0)
            
    def set_initial(self, pose):
        init_node = self.closest_node(pose)
        self.graph['initial'] = set([init_node])
        return init_node

    def closest_node(self, pose):
        node = min(list(self.nodes()), key= lambda n: distance(n,pose))
        return node

class MotionFts(DiGraph):
    def __init__(self, node_dict, symbols, ts_type):
        DiGraph.__init__(self, symbols=symbols, type=ts_type, initial=set())
        
        self.new_reg_count = 0
        
        for (n, label) in node_dict.items():
            self.add_node(n, label=label, status='confirmed')
            

    def add_un_edges(self, edge_list, unit_cost=1):
        for edge in edge_list:
            f_node = edge[0]
            t_node = edge[1]
            dist = distance(f_node, t_node)
            self.add_edge(f_node, t_node, weight=dist*unit_cost)
            self.add_edge(t_node, f_node, weight=dist*unit_cost)
        for node in list(self.nodes()):
            #self.add_edge(node, node, weight=unit_cost)
            # allow self-transit to 0-cost
            self.add_edge(node, node, weight=0)

    def add_full_edges(self,unit_cost=1):
        for f_node in list(self.nodes()):
            for t_node in list(self.nodes()):
                dist = distance(f_node, t_node)
                if (f_node, t_node) not in self.edges():
                    self.add_edge(f_node, t_node, weight=dist*unit_cost)

    def set_initial(self, pose):
        init_node = self.closest_node(pose)
        self.graph['initial'] = set([init_node])
        return init_node

    def closest_node(self, pose):
        node = min(list(self.nodes()), key= lambda n: distance(n,pose))
        return node

    '''
    def update_after_region_change(self, sense_info, com_info, margin=10):
        # sense_info = {'label':set((x,y), l', l'_)), 'edge':(set(add_edges), set(del_edges))}
        # com_info = set((x,y), l', l'_))
        # margin for adding new nodes, NOTE units!
        changed_regs = set()
        # label udpate
        label_info = sense_info['label']
        label_info.update(com_info)
        for mes in label_info:
            if mes[1]:
                close_node = self.closest_node(mes[0])
                if distance(close_node, mes[0])>margin:
                    self.add_node(mes[0], mes[1])
                else:
                    old_label = self.node[close_node]['label']
                    new_label = old_label.union(mes[1]).difference(mes[2])
                    if old_label != new_label:
                        self.node[close_node]['label'] = set(new_label)
                        self.node[close_node]['status'] = 'notconfirmed'
                        changed_regs.add(close_node)
        # edges udpate
        edge_info = sense_info['edge']
        for e in edge_info[0]:
            self.add_edge(e[0], e[1], weight=distance(e[0], e[1]))
            self.node[close_node]['status'] = 'notconfirmed'
            changed_regs.add(e[0])
        for e in edge_info[1]:
            self.remove_edge(e[0], e[1])
            changed_regs.add(e[0])
            self.node[close_node]['status'] = 'notconfirmed'
        return changed_regs
    '''
    def update_after_region_change(self, sense_info, margin = 10):
        # sense_info = {'label':set(region)} 
        reg = sense_info['label']
        #close_node = self.closest_node(reg)
        #if distance(close_node,reg) > margin:
        label = set(['{}'.format(self.new_reg_count),])
        self.new_reg_count += 1
        self.add_node(tuple(reg[0]), label = label, status='confirmed')
        self.add_edge(tuple(reg[0]), tuple(reg[0]), weight=0)
        print('region graph updated: %d states and %s transitions' %(len(self.nodes), len(self.edges)))
    
    def update_after_edges_change(self, sense_info):
        # sense_info = {'edge':(set(add_edges), set(del_edges))} 
        edge_info = sense_info['edge']
        for e in edge_info[0]:
            self.add_edge(e[0][0], e[1][0], weight=distance(e[0][0], e[1][0]))
            self.add_edge(e[1][0], e[0][0], weight=distance(e[0][0], e[1][0]))
        for e in edge_info[1]:
            self.remove_edge(e[0], e[1])
            self.remove_edge(e[1], e[0])
        print('region graph updated: %d states and %s transitions' %(len(self.nodes), len(self.edges)))
            
class ActionModel(object):
    # action_dict = {act_name: (cost, guard_formula, label)}
    def __init__(self, action_dict):
        self.raw = action_dict
        self.action = dict()
        for act_name, attrib in action_dict.items():
            cost = attrib[0]
            guard_formula = attrib[1]
            guard_expr = parse_guard(guard_formula)
            label = attrib[2]
            self.action[act_name] = (cost, guard_expr, label)
        self.action['None'] = (0, parse_guard('1'), set()) 

    def allowed_actions(self, ts_node_label):
        allow_action = set()
        for act_name, attrib in self.action.items():
            if (attrib[1].check(ts_node_label)):
                allow_action.add(act_name)
        return allow_action



class MotActModel(DiGraph):
    def __init__(self, mot_fts, act_model):
        DiGraph.__init__(self, region=mot_fts, action=act_model, initial=set(), type='MotActModel')

    def composition(self, reg, act):
        prod_node = (reg, act)
        if not self.has_node(prod_node):
            new_label = self.graph['region'].nodes[reg]['label'].union(self.graph['action'].action[act][2])
            self.add_node(prod_node, label=new_label, region=reg, action=act, marker='unvisited')
            if ((reg in self.graph['region'].graph['initial']) and (act == 'None')):
                self.graph['initial'].add(prod_node)
        return prod_node

    def projection(self, prod_node):
        reg = self.node[prod_node]['region']
        act = self.node[prod_node]['action']
        return reg, act

    def build_initial(self):
        for reg_init in self.graph['region'].graph['initial']:
            init_prod_node = self.composition(reg_init, 'None')

    def build_full(self):
        for reg in list(self.graph['region'].nodes()):
            for act in self.graph['action'].action:
                prod_node = self.composition(reg, act)
                # # actions
                if (act == 'None'):
                    label = self.graph['region'].nodes[reg]['label']
                    for act_to in self.graph['action'].allowed_actions(label):
                        prod_node_to = self.composition(reg, act_to)
                        if act_to != 'None':
                            self.add_edge(prod_node, prod_node_to, weight=self.graph['action'].action[act_to][0], label= act_to, marker= 'visited')
                        else:
                            self.add_edge(prod_node, prod_node_to, weight=self.graph['action'].action[act_to][0], label= 'goto', marker= 'visited')
                # motions
                for reg_to in list(self.graph['region'].successors(reg)):
                    prod_node_to = self.composition(reg_to, 'None')
                    self.add_edge(prod_node, prod_node_to, weight=self.graph['region'][reg][reg_to]['weight'], label= 'goto', marker= 'visited')
        print ('full_model constructed with %d states and %s transitions' %(len(self.nodes()), len(self.edges())) )
    
    # -----------------------------------
    # CHANGE REGIONS
    def update_after_region_change(self, sense_info):
            reg = sense_info['label']
            reg = (tuple(reg[0]))
            for act in self.graph['action'].action:
                prod_node = self.composition(reg, act)
                # # actions
                if (act == 'None'):
                    label = self.graph['region'].nodes[reg]['label']
                    for act_to in self.graph['action'].allowed_actions(label):
                        prod_node_to = self.composition(reg, act_to)
                        if act_to != 'None':
                            self.add_edge(prod_node, prod_node_to, weight=self.graph['action'].action[act_to][0], label= act_to, marker= 'visited')
                        else:
                            self.add_edge(prod_node, prod_node_to, weight=self.graph['action'].action[act_to][0], label= 'goto', marker= 'visited')
                # motions
                for reg_to in list(self.graph['region'].successors(reg)):
                    prod_node_to = self.composition(reg_to, 'None')
                    self.add_edge(prod_node, prod_node_to, weight=self.graph['region'][reg][reg_to]['weight'], label= 'goto', marker= 'visited')
            print('model updated with %d states and %s transitions' %(len(self.nodes), len(self.edges)))
        
    # CHANGE EDGES    
    def update_after_edges_change(self, sense_info):
        edge_info = sense_info['edge']
        for e in edge_info[0]:
            prod_node = e[0]
            prod_node_to = e[1]
            reg = prod_node[0]
            reg_to = prod_node_to[0]
            self.add_edge(prod_node, prod_node_to, weight = self.graph['region'].edges[reg,reg_to]['weight'], label = 'goto', marker = 'visited')
            self.add_edge(prod_node_to, prod_node, weight = self.graph['region'].edges[reg_to,reg]['weight'], label = 'goto', marker = 'visited')
        print('model updated with %d states and %s transitions' %(len(self.nodes), len(self.edges)))
    
    def fly_successors_iter(self, prod_node): 
        reg, act = self.projection(prod_node)
        # been visited before, and hasn't changed 
        if ((self.node[prod_node]['marker'] == 'visited') and 
            (self.graph['region'].node[self.node[prod_node]['region']]['status'] == 'confirmed')):
            for prod_node_to in list(self.successors(prod_node)):
                yield prod_node_to, self.edge[prod_node][prod_node_to]['weight']
        else:
            self.remove_edges_from(self.out_edges(prod_node))
            # actions 
            label = self.graph['region'].node[reg]['label']
            for act_to in self.graph['action'].allowed_actions(label):
                prod_node_to = self.composition(reg, act_to)
                cost = self.graph['action'].action[act_to][0]
                self.add_edge(prod_node, prod_node_to, weight=cost, label= act_to)
                yield prod_node_to, cost
            # motions
            for reg_to in list(self.graph['region'].successors(reg)):
                if reg_to != reg:
                    prod_node_to = self.composition(reg_to, 'None')
                    cost = self.graph['region'][reg][reg_to]['weight']
                    self.add_edge(prod_node, prod_node_to, weight=cost, label= 'goto')         
                    yield prod_node_to, cost
            self.graph['region'].node[self.node[prod_node]['region']]['status'] = 'confirmed'
            self.node[prod_node]['marker'] = 'visited'
        print ('full FTS model constructed with %d states and %s transitions' %(len(self.nodes()), len(self.edges())))          

    def fly_predecessors_iter(self, prod_node): 
        reg, act = self.projection(prod_node)
        # actions
        label = self.graph['region'].node[reg]['label']
        if act in self.graph['action'].allowed_actions(label):    
            for f_act in self.graph['action'].action.iterkeys():
                f_prod_node = self.composition(reg, f_act)
                cost = self.graph['action'].action[act][0]
                self.add_edge(f_prod_node, prod_node, weight=cost, label= act)
                yield f_prod_node, cost
        # motions
        if act == 'None':
            for f_reg in self.graph['region'].predecessors_iter(reg):
                if f_reg !=reg:
                    for f_act in self.graph['action'].action.iterkeys():
                            f_prod_node = self.composition(f_reg, f_act)
                            cost = self.graph['region'][f_reg][reg]['weight']
                            self.add_edge(f_prod_node, prod_node, weight=cost, label= 'goto')         
                            yield f_prod_node, cost
