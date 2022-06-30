# -*- coding: utf-8 -*-

from ltl_tools.buchi import check_label_for_buchi_edge

from networkx.classes.digraph import DiGraph


class ProdAut(DiGraph):
    def __init__(self, ts, buchi, beta=100):
        DiGraph.__init__(self, ts=ts, buchi=buchi, beta=beta, initial=set(), accept=set(), type='ProdAut')

    def build_full(self):
        for f_ts_node in self.graph['ts'].nodes:
            for f_buchi_node in self.graph['buchi'].nodes:
                f_prod_node = self.composition(f_ts_node, f_buchi_node)
                #print 'f_prod_node' , (f_ts_node, f_buchi_node)
                for t_ts_node in list(self.graph['ts'].successors(f_ts_node)):
                    for t_buchi_node in list(self.graph['buchi'].successors(f_buchi_node)):
                            t_prod_node = self.composition(t_ts_node, t_buchi_node)
                            #print 't_prod_node' , (t_ts_node, t_buchi_node)
                            label = self.graph['ts'].nodes[f_ts_node]['label']
                            cost = self.graph['ts'].edges[f_ts_node, t_ts_node]['weight']
                            truth, dist = check_label_for_buchi_edge(self.graph['buchi'], label, f_buchi_node, t_buchi_node)
                            #print 'label,truth,total_weight', label,truth,total_weight
                            if truth:
                                                                total_weight = cost + self.graph['beta']*dist
                                                                self.add_edge(f_prod_node, t_prod_node, weight=total_weight, cost=cost, distance=dist)
                                                                #print 'add edge', (f_prod_node, t_prod_node)
        print ('full product constructed with %d states and %s transitions' %(len(self.nodes), len(self.edges)))
  
    # --------------------------------------------
    # UPDATE PRODUCT AFTER A NEW EDGE HAS BEEN DISCOVERED      
    def update_after_edges_change(self, sense_info):
        # sense_info = {'label': set((x,y), l', l'_)), 'edge':(set(add_edges), set(del_edges))} 
        edge_info = sense_info['edge']
        for e in edge_info[0]:
            f_ts_node = e[0]
            for f_buchi_node in self.graph['buchi'].nodes:
                f_prod_node = self.composition(f_ts_node, f_buchi_node)
                t_ts_node = e[1]
                for t_buchi_node in list(self.graph['buchi'].successors(f_buchi_node)):
                    t_prod_node = self.composition(t_ts_node, t_buchi_node)
                    label = self.graph['ts'].nodes[f_ts_node]['label']
                    cost = self.graph['ts'].edges[f_ts_node, t_ts_node]['weight']
                    truth, dist = check_label_for_buchi_edge(self.graph['buchi'], label, f_buchi_node, t_buchi_node)
                    if truth:
                        total_weight = cost + self.graph['beta']*dist
                        self.add_edge(f_prod_node, t_prod_node, weight = total_weight, cost = cost, distance = dist)
                        self.add_edge(t_prod_node, f_prod_node, weight = total_weight, cost = cost, distance = dist)
        print('product updated with %d states and %s transitions' %(len(self.nodes), len(self.edges)))
        
    # UPDATE PRODUCT AFTER A NEW REGION HAS BEEN DISCOVERED
    def update_after_region_change(self, sense_info):
        f_ts_node = sense_info['label']
        for f_buchi_node in self.graph['buchi'].nodes:
                f_prod_node = self.composition(f_ts_node, f_buchi_node)
                #print 'f_prod_node' , (f_ts_node, f_buchi_node)
                for t_ts_node in list(self.graph['ts'].successors(f_ts_node)):
                    for t_buchi_node in list(self.graph['buchi'].successors(f_buchi_node)):
                            t_prod_node = self.composition(t_ts_node, t_buchi_node)
                            #print 't_prod_node' , (t_ts_node, t_buchi_node)
                            label = self.graph['ts'].nodes[f_ts_node]['label']
                            cost = self.graph['ts'].edges[f_ts_node, t_ts_node]['weight']
                            truth, dist = check_label_for_buchi_edge(self.graph['buchi'], label, f_buchi_node, t_buchi_node)
                            #print 'label,truth,total_weight', label,truth,total_weight
                            if truth:
                                                                total_weight = cost + self.graph['beta']*dist
                                                                self.add_edge(f_prod_node, t_prod_node, weight=total_weight, cost=cost, distance=dist)
                                                                #print 'add edge', (f_prod_node, t_prod_node)
        print('product updated with %d states and %s transitions' %(len(self.nodes), len(self.edges)))
    # -----------------------------------------------
        
                                              
    def build_full_margin(self, opt_path):
                if len(opt_path) >= 2:
                        opt_edges = zip(opt_path[0::2], opt_path[1::2])
                for f_ts_node in self.graph['ts'].nodes():
                    for f_buchi_node in self.graph['buchi'].nodes():
                        f_prod_node = self.composition(f_ts_node, f_buchi_node)
                        #print 'f_prod_node' , (f_ts_node, f_buchi_node)
                        for t_ts_node in list(self.graph['ts'].successors(f_ts_node)):
                            for t_buchi_node in list(self.graph['buchi'].successors(f_buchi_node)):
                                t_prod_node = self.composition(t_ts_node, t_buchi_node)
                                #print 't_prod_node' , (t_ts_node, t_buchi_node)
                                label = self.graph['ts'].nodes[f_ts_node]['label']
                                cost = self.graph['ts'].edges[f_ts_node, t_ts_node]['weight']
                                truth, dist = check_label_for_buchi_edge(self.graph['buchi'], label, f_buchi_node, t_buchi_node)
                                total_weight = cost + self.graph['beta']*dist + 1
                                #print 'label,truth,total_weight', label,truth,total_weight
                                if (f_prod_node, t_prod_node) in opt_edges:
                                    k = 1
                                else:
                                    k = 0
                                total_weight -= k
                                if truth:
                                    self.add_edge(f_prod_node, t_prod_node, weight=total_weight, cost=cost, distance=dist)
                                    #print 'add edge', (f_prod_node, t_prod_node)
                print ('full product constructed with %d states and %s transitions' %(len(self.nodes), len(self.edges)))
                
    def composition(self, ts_node, buchi_node):
        prod_node = (ts_node, buchi_node)
        if not self.has_node(prod_node):
            self.add_node(prod_node, ts=ts_node, buchi=buchi_node, marker='unvisited')
            if ((ts_node in self.graph['ts'].graph['initial']) and
                (buchi_node in self.graph['buchi'].graph['initial'])):
                self.graph['initial'].add(prod_node)
            if (buchi_node in self.graph['buchi'].graph['accept']):
                self.graph['accept'].add(prod_node)
        return prod_node

    def projection(self, prod_node):
        ts_node = self.node[prod_node]['ts']
        buchi_node = self.node[prod_node]['buchi']
        return ts_node, buchi_node

    def build_initial(self):
        self.graph['ts'].build_initial()
        for ts_init in self.graph['ts'].graph['initial']:
            for buchi_init in self.graph['buchi'].graph['initial']:
                init_prod_node = self.composition(ts_init, buchi_init)

    def build_accept(self):
        self.graph['ts'].build_full()
        accept = set()
        for ts_node in self.graph['ts'].nodes:
            for buchi_accept in self.graph['buchi'].graph['accept']:
                accept_prod_node = self.composition(ts_node, buchi_accept)

    def accept_predecessors(self, accept_node):
        pre_set = set()
        t_ts_node, t_buchi_node = self.projection(accept_node)
        for f_ts_node, cost in self.graph['ts'].fly_predecessors_iter(t_ts_node):
            for f_buchi_node in self.graph['buchi'].predecessors_iter(t_buchi_node):
                f_prod_node = self.composition(f_ts_node, f_buchi_node)
                label = self.graph['ts'].node[f_ts_node]['label']
                truth, dist = check_label_for_buchi_edge(self.graph['buchi'], label, f_buchi_node, t_buchi_node)
                total_weight = cost + self.graph['beta']*dist
                if truth:
                    pre_set.add(f_prod_node)
                    self.add_edge(f_prod_node, accept_node, weight=total_weight)
        return pre_set

    def fly_successors_iter(self, f_prod_node):
        f_ts_node, f_buchi_node = self.projection(f_prod_node)
        # been visited before, and hasn't changed 
        if ((self.nodes[f_prod_node]['marker'] == 'visited') and 
            (self.graph['ts'].graph['region'].nodes[
                self.graph['ts'].nodes[self.node[f_prod_node]['ts']]['region']]['status'] == 'confirmed')):
            for t_prod_node in list(self.successors(f_prod_node)):
                yield t_prod_node, self.edge[f_prod_node][t_prod_node]['weight']
        else:
            self.remove_edges_from(self.out_edges(f_prod_node))
            for t_ts_node,cost in list(self.graph['ts'].fly_successors(f_ts_node)):
                for t_buchi_node in list(self.graph['buchi'].successors(f_buchi_node)):
                    t_prod_node = self.composition(t_ts_node, t_buchi_node)
                    label = self.graph['ts'].node[f_ts_node]['label']
                    truth, dist = check_label_for_buchi_edge(self.graph['buchi'], label, f_buchi_node, t_buchi_node)
                    total_weight = cost + self.graph['beta']*dist
                    if truth:
                        self.add_edge(f_prod_node, t_prod_node, weight=total_weight)
                        yield t_prod_node, total_weight
            self.node[f_prod_node]['marker'] = 'visited'


class ProdAut_Run(object):
    # prefix, suffix in product run
    # prefix: init --> accept, suffix accept --> accept
    # line, loop in ts
    def __init__(self, product, prefix, precost, suffix, sufcost, totalcost):
        self.prefix = prefix
        self.precost = precost
        self.suffix = suffix
        self.sufcost = sufcost
        self.totalcost = totalcost
        #self.prod_run_to_prod_edges(product)
        self.plan_output(product)
        #self.plan = chain(self.line, cycle(self.loop))
        #self.plan = chain(self.loop)

    def prod_run_to_prod_edges(self, product):
        self.pre_prod_edges = zip(self.prefix[0:-2], self.prefix[1:-1])
        self.suf_prod_edges = zip(self.suffix[0:-2], self.suffix[1:-1])

    def plan_output(self, product):
        self.line = [product.nodes[node]['ts'] for node in self.prefix]
        self.loop = [product.nodes[node]['ts'] for node in self.suffix]
        if len(self.line) == 2:
                        self.pre_ts_edges = [(self.line[0], self.line[1])]
        else:
                self.pre_ts_edges = zip(self.line[0:-1], self.line[1:])
        if len(self.loop) == 2:
                        self.suf_ts_edges = [(self.loop[0], self.loop[1])]
        else:
                self.suf_ts_edges = list(zip(self.loop[0:-1], self.loop[1:]))
                self.suf_ts_edges.append((self.loop[-1],self.loop[0]))
        # output plan
        self.pre_plan = []
        self.pre_plan.append(self.line[0][0]) 
        for ts_edge in self.pre_ts_edges:
            if product.graph['ts'][ts_edge[0]][ts_edge[1]]['label'] == 'goto':
                self.pre_plan.append(ts_edge[1][0]) # motion 
            else:
                self.pre_plan.append(ts_edge[1][1]) # action
        bridge = (self.line[-1],self.loop[0])
        if product.graph['ts'][bridge[0]][bridge[1]]['label'] == 'goto':
            self.pre_plan.append(bridge[1][0]) # motion 
        else:
            self.pre_plan.append(bridge[1][1]) # action
        self.suf_plan = []		
        for ts_edge in self.suf_ts_edges:
            if product.graph['ts'][ts_edge[0]][ts_edge[1]]['label'] == 'goto':
                self.suf_plan.append(ts_edge[1][0]) # motion 
            else:
                self.suf_plan.append(ts_edge[1][1]) # action
           
    def update_line(self, product, new_line, new_precost):
        self.precost = new_precost
        self.line = new_line
        if len(self.line) == 2:
            self.pre_ts_edges = [(self.line[0], self.line[1])]
        else:
            self.pre_ts_edges = zip(self.line[0:-1], self.line[1:])

        self.pre_plan = []
        self.pre_plan.append(self.line[0][0]) 
        for ts_edge in self.pre_ts_edges:
            if product.graph['ts'].edges[ts_edge[0], ts_edge[1]]['label'] == 'goto':
                self.pre_plan.append(ts_edge[1][0]) # motion 
            else:
                self.pre_plan.append(ts_edge[1][1]) # action
        bridge = (self.line[-1],self.loop[0])
        if product.graph['ts'].edges[bridge[0], bridge[1]]['label'] == 'goto':
            self.pre_plan.append(bridge[1][0]) # motion 
        else:
            self.pre_plan.append(bridge[1][1]) # action 








