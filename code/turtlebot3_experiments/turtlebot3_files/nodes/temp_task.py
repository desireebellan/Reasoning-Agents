#!/usr/bin/env python

import rospy
from turtlebot3_files.msg import Task
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
  
msg = """
Insert Temporal Task
In the form :
initialState goalState timeOfExecution
CTRL-C to quit
"""

def get_input():
     keypress = input().split(' ')
     return keypress[0], keypress[1], float(keypress[2])
if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node('temp_task')
    pub = rospy.Publisher('temp_task', Task, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "hotel")
    
    try:
      while(1):
        print(msg)
        task = Task()
        state, goal, tsg = get_input()
        if turtlebot3_model == "hotel":
          ap = ['r0', 'r1', 'r2',
                'r3', 'r4', 'r5',
                'r6', 'r7', 'r8',
                'c1', 'c2', 'c3',
                'c4']
          loc = [(-6.27, -2.30, 0.00), (-1.35, -2.73 , 0.00), (1.66, -2.24, 0.00),
                (5.86, -1.76, 0.00), (-1.99, 0.04, 0.00), (3.03, 0.00, 0.00),
                (6.75, -0.01, 0.00), (-5.64, 2.66, 0.00), (1.12, 2.65, 0.00),
                (-6.3, 0.8, 0.00), (0.62, 0.2, 0.00), (5.35, 0.00, 0.00),
                (-1.1, 1.9, 0.0),
                  ]
        elif turtlebot3_model == "hospital":
          ap = ['r0', 'r1', 'r2',
                'r3', 'r4', 'r5',
                'r6','c1', 'c2', 'c3',
                'c4']
          loc = [(3.4, 0.00 , 0.00), (-4.00, 5.38, 0.00), (-7.00, 5.38, 0.00),
                (-10.00, 5.38, 0.00), (-10.00, -2.27, 0.00), (-7.00, -2.27, 0.00),
                (-4.00, -2.27, 0.00), (0.00, 0.00, 0.00), (-4.00, 0.00, 0.00),
                (-7.00, 0.00, 0.00), (-10.00, 0.00, 0.00)
                ] 
        if state not in ap or goal not in ap:
                print('State or Goal not in the current model')
        else:
            s = loc[ap.index(state)]
            g = loc[ap.index(goal)]
            task.sx = s[0]; task.sy = s[1]; task.gx = g[0]; task.gy = g[1]; task.tsg = tsg
      
            pub.publish(task)
    
    except Exception as e:
      print(e)
    
    
    
    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
