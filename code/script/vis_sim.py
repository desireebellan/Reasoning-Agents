from math import exp

import matplotlib
import matplotlib.pyplot as plt

from PIL import Image

import pickle

matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True


def draw_map(img_dir):
    im = Image.open(img_dir)
    pix = im.load()
    Nx = im.size[0]
    Ny = im.size[1]
    scale = 0.02
    # print im.size
    # x = 100
    # y= 100
    # print pix[x,y] #Get the RGBA Value of the a pixel of an image
    # print pix[0,0]
    # (1000, 450)
    # (255, 255, 255)
    # (0, 0, 0)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    for nx in list(range(0, Nx, 5))+[Nx-1]:
        for ny in list(range(0, Ny, 5))+[Ny-1]:
            if pix[nx,ny][0] == 0:
                ax.plot(nx*scale, (Ny-ny)*scale, color='k',
                        marker='s', markersize=1)
    ax.set_xlim([0,(Nx-1)*scale])
    plt.axis('off')
    plt.axis('equal')
    fig.tight_layout(pad=0)
    fig.savefig('map.pdf',bbox_inches='tight', pad_inches=0)
    return fig

def plot_traj_init(img_dir, A_robot_pose, A_control, output_name = 'turtlebot3_hotel'):
    robot_x = []
    robot_y = []
    robot_x_part = []
    robot_y_part = []
    robot_x_hil = []
    robot_y_hil = []
    robot_x_hil_part = []
    robot_y_hil_part = []
    sample = 5
    robot = False
    
    if 'hotel' in output_name:
        L = range(len(A_robot_pose)-1)
        for k in L[0::sample]:
            robot_pose = A_robot_pose[k]
            h_control = A_control[k][0]
            if (abs(h_control[0])+abs(h_control[1]))>0.1:
                if robot and len(robot_x_part) != 0:
                    robot_x.append(robot_x_part)
                    robot_y.append(robot_y_part)
                    robot_x_part = []
                    robot_y_part = []                  
                robot_x_hil_part.append(robot_pose[1][0] + 7.5)
                robot_y_hil_part.append(robot_pose[1][1] + 3.0)   
                robot = False     
            else:            
                if not robot and len(robot_x_hil_part)!=0:
                    robot_x_hil.append(robot_x_hil_part)
                    robot_y_hil.append(robot_y_hil_part)
                    robot_x_hil_part = []
                    robot_y_hil_part = [] 
                robot_x_part.append(robot_pose[1][0] + 7.5)
                robot_y_part.append(robot_pose[1][1] + 3.0)
                robot = True
        if len(robot_x_part)!=0:
            robot_x.append(robot_x_part)
            robot_y.append(robot_y_part)
        if len(robot_x_hil_part)!=0:
            robot_x_hil.append(robot_x_hil_part)
            robot_y_hil.append(robot_y_hil_part)
            
        im = Image.open(img_dir)
        im = im.resize((750,300))
        pix = im.load()
        Nx = im.size[0]
        Ny = im.size[1]
        scale = 0.02
        Ns = 2
        fig = plt.figure()
        ax1 = fig.add_subplot(121)
        for nx in list(range(0, Nx, Ns))+[Nx-1]:
            for ny in list(range(0, Ny, Ns))+[Ny-1]:
                if pix[nx,ny][0] == 0:
                    ax1.plot(nx*scale, (Ny-ny)*scale, color='k',
                            marker='s', markersize=1)
        # plot pre hi
        ax1.plot(robot_x[0], robot_y[0], color='b',
                    linestyle='-',linewidth=2, marker='o', mfc='grey',
                    fillstyle='full', markersize=2.3, zorder=2, label = r'$\tau_r^0$')
        if len(robot_x_hil)!=0:
            ax1.plot(robot_x_hil[0], robot_y_hil[0], color = 'r',
                    linestyle='-',linewidth=2, marker='o', mfc='grey',
                    fillstyle='full', markersize=2.3, zorder=3, label = r'HIL')
        for idx in range(1, len(robot_x)):
            ax1.plot(robot_x[idx], robot_y[idx], color='b',
                    linestyle='-',linewidth=2, marker='o', mfc='grey',
                    fillstyle='full', markersize=2.3, zorder=2)
        for idx in range(1, len(robot_x_hil)):
            ax1.plot(robot_x_hil[idx], robot_y_hil[idx], color = 'r',
                    linestyle='-',linewidth=2, marker='o', mfc='grey',
                    fillstyle='full', markersize=2.3, zorder=3)
        
        # plot regions
        ap = ['r_0', 'r_1', 'r_2',
            'r_3', 'r_4', 'r_5',
            'r_6', 'r_7', 'r_8',
            'c_1', 'c_2', 'c_3',
            'c_4']
        roi = [(-6.27, -2.30, 0.45), (-1.04, -2.53 , 0.45), (1.66, -2.24, 0.45),
        (5.82, -1.99, 0.45), (-1.99, 0.04, 0.45), (3.03, 0.00, 0.45),
        (6.75, -0.01, 0.45), (-5.64, 2.66, 0.45), (1.12, 2.65, 0.45),
        (-6.3, 0.8, 0.75), (0.62, 0.2, 0.75), (5.35, 0.00, 0.75),
        (-1.1, 1.9, 0.75)
            ]
        for k in range(len(roi)):
            reg = list(roi[k])
            reg[0] = reg[0] + 7.5
            reg[1] = reg[1] + 3.0
            reg = tuple(reg)
            rec = matplotlib.patches.Rectangle((reg[0]-reg[2], reg[1]-reg[2]), reg[2]*2, reg[2]*2, fill = True, facecolor = 'cyan', edgecolor = 'black', linewidth = 1,  alpha =0.8)
            ax1.add_patch(rec)
            ax1.text(reg[0], reg[1]-0.1, r'$%s$' %ap[k], fontsize = 13, fontweight = 'bold', zorder = 3)
    
    elif 'hostpital' in output_name:
        L = range(len(A_robot_pose)-1)
        for k in L[0::sample]:
            robot_pose = A_robot_pose[k]
            h_control = A_control[k][0]
            if (abs(h_control[0])+abs(h_control[1]))>0.1:
                robot_x_hil.append(robot_pose[1][0])
                robot_y_hil.append(robot_pose[1][1])          
            else:
                robot_x.append(robot_pose[1][0])
                robot_y.append(robot_pose[1][1])
        im = Image.open(img_dir)
        pix = im.load()
        Nx = im.size[0]
        Ny = im.size[1]
        scale = 0.02
        Ns = 2
        fig = plt.figure()
        ax1 = fig.add_subplot(121)
        for nx in list(range(0, Nx, Ns))+[Nx-1]:
            for ny in list(range(0, Ny, Ns))+[Ny-1]:
                if pix[nx,ny][0] == 0:
                    ax1.plot(nx*scale, (Ny-ny)*scale, color='k',
                            marker='s', markersize=1)
        # plot pre hi
        print('traj length {}'.format(len(robot_x)))
        ax1.plot(robot_x, robot_y, color='b',
                linestyle='-',linewidth=2, marker='o', mfc='grey',
                fillstyle='full', markersize=2.3, zorder=2)
        
        # plot regions
        ap = ['r0', 'r1', 'r2',
            'r3', 'r4', 'r5',
            'r6','c1', 'c2', 'c3',
            'c4']
        roi = [(3.4, 0.00 , 0.00), (-4.00, 5.38, 0.00), (-7.00, 5.38, 0.00),
            (-10.00, 5.38, 0.00), (-10.00, -2.27, 0.00), (-7.00, -2.27, 0.00),
            (-4.00, -2.27, 0.00), (0.00, 0.00, 0.00), (-4.00, 0.00, 0.00),
            (-7.00, 0.00, 0.00), (-10.00, 0.00, 0.00)
                ]
        for k in range(len(roi)):
            reg = list(roi[k])
            reg[0] = reg[0] + 7.0
            reg[1] = reg[1] + 3.0
            reg = tuple(reg)
            rec = matplotlib.patches.Rectangle((reg[0]-reg[2], reg[1]-reg[2]), reg[2]*2, reg[2]*2, fill = True, facecolor = 'cyan', edgecolor = 'black', linewidth = 1,  alpha =0.8)
            ax1.add_patch(rec)
            ax1.text(reg[0], reg[1]-0.1, r'$%s$' %ap[k], fontsize = 13, fontweight = 'bold', zorder = 3)
        
    ax1.legend(ncol=1,bbox_to_anchor=(0.78,0.56),loc='lower left', borderpad=0.1, labelspacing=0.2, columnspacing= 0.3, numpoints=3, prop={'size': 10})        
    ax1.grid()
    ax1.set_xlim([0,(Nx-1)*scale])
    ax1.axis('off')
    ax1.axis('equal')
    fig.savefig(output_name,bbox_inches='tight', pad_inches=0)
    
def plot_traj_irl(img_dir, A_robot_pose, A_control, output_name = 'turtlebot3_hotel_irl'):
    robot_x = []
    robot_y = []
    robot_x_hil = []
    robot_y_hil = []
    sample = 5
    if 'hotel' in output_name:
        L = range(len(A_robot_pose)-1)
        for k in L[0::sample]:
            robot_pose = A_robot_pose[k]
            h_control = A_control[k][0]
            if (abs(h_control[0])+abs(h_control[1]))>0.1:
                robot_x_hil.append(robot_pose[1][0] + 7.5)
                robot_y_hil.append(robot_pose[1][1] + 3.0)          
            else:
                robot_x.append(robot_pose[1][0] + 7.5)
                robot_y.append(robot_pose[1][1] + 3.0)
        
        im = Image.open(img_dir)
        im = im.resize((750,300))
        pix = im.load()
        Nx = im.size[0]
        Ny = im.size[1]
        scale = 0.02
        Ns = 2
        fig = plt.figure()
        
        ax2 = fig.add_subplot(122)
        for nx in list(range(0, Nx, Ns))+[Nx-1]:
            for ny in list(range(0, Ny, Ns))+[Ny-1]:
                if pix[nx,ny][0] == 0:
                    ax2.plot(nx*scale, (Ny-ny)*scale, color='k',
                            marker='s', markersize=1)
                    

    
'''
def plot_traj(img_dir, A_robot_pose, A_control, output_name = 'turtlebot3_hotel'):
    # plot pre hi
    print('traj length {}'.format(len(robot_x)))
    k1 = [0, int(100/sample)] # initial 
    k2 = [int(100/sample), int(250/sample)] #hi
    k3 = [int(250/sample),int(1110/sample)] # normal
    k4 = [int(1110/sample), int(1200/sample)] #hi
    k5 = [int(1200/sample), int(2110/sample)] #update
    k6 = [int(2110/sample), int(2390/sample)] #temp    
    ax2.plot(robot_x[k4[0]:k4[1]], robot_y[k4[0]:k4[1]], color='r',
            linestyle='-',linewidth=2, marker='o', mfc='grey',
             fillstyle='full', markersize=2.3, zorder=6, label=r'HIL')
    ax2.plot(robot_x[k5[0]:k5[1]], robot_y[k5[0]:k5[1]], color='g',
            linestyle='-',linewidth=2, marker='o', mfc='grey',
            fillstyle='full', markersize=2.3, zorder=5, label=r'$\tau_r^t$')
    ax2.plot(robot_x[k6[0]:k6[1]], robot_y[k6[0]:k6[1]], color='m',
            linestyle='-',linewidth=2, marker='o', mfc='grey',
             fillstyle='full', markersize=2.3, zorder=7, label=r'$\varphi_{\textup{temp}}$')            
    ax2.plot(robot_x[(k6[1]-10):], robot_y[(k6[1]-10):], color='g',
            linestyle='-',linewidth=2, marker='o', mfc='grey',
            fillstyle='full', markersize=2.3, zorder=5)    
    #---------- print regions of interest
    ap = ['r_0', 'r_1', 'r_2',
          'r_3', 'r_4', 'r_5',
          'r_6', 'r_7', 'r_8',
          'c_1', 'c_2', 'c_3',
          'c_4']
    roi = [(-6.27, -2.30, 0.00), (-1.04, -2.53 , 0.00), (1.66, -2.24, 0.00),
       (5.82, -1.99, 0.00), (-1.99, 0.04, 0.00), (3.03, 0.00, 0.00),
       (6.75, -0.01, 0.00), (-5.64, 2.66, 0.00), (1.12, 2.65, 0.00),
       (-6.3, 0.8, 0.00), (0.62, 0.2, 0.00), (5.35, 0.00, 0.00),
       (-1.1, 1.9, 1.0)
         ]
    for k in range(len(roi)):
        reg = list(roi[k])
        reg[0] = reg[0] + 7.0
        reg[1] = reg[1] + 3.0
        reg = tuple(reg)
        rec = matplotlib.patches.Rectangle((reg[0]-reg[2], reg[1]-reg[2]), reg[2]*2, reg[2]*2, fill = True, facecolor = 'cyan', edgecolor = 'black', linewidth = 1,  alpha =0.8)
        ax2.add_patch(rec)
        ax2.text(reg[0], reg[1]-0.1, r'$%s$' %ap[k], fontsize = 15, fontweight = 'bold', zorder = 3)
    ax2.legend(ncol=1,bbox_to_anchor=(0.78,0.56),loc='lower left', borderpad=0.1, labelspacing=0.1, columnspacing= 0.1, numpoints=3, prop={'size': 7.8})        
    ax2.grid()
    ax2.set_xlim([0,(Nx-1)*scale])
    ax2.axis('off')
    ax2.axis('equal')
    #plt.tight_layout(pad=0)
    fig.savefig(output_name,bbox_inches='tight', pad_inches=0)
    '''
        
    
def plot_control(A_control):
    c_linear = []
    c_angular = []
    h_linear = []
    h_angular = []
    m_linear = []
    m_angular = []
    for control in A_control:
        [tele_control, navi_control, mix_control] = control
        c_linear.append(navi_control[0])
        c_angular.append(navi_control[1])
        h_linear.append(tele_control[0])
        h_angular.append(tele_control[1])
        m_linear.append(mix_control[0])
        m_angular.append(mix_control[1])
    #------------------------------ plot v
    step = 1.0
    T = [t*step for t in range(len(A_control))]
    fig = plt.figure(figsize=(10,3))
    ax = fig.add_subplot(111)
    ax.plot(T, c_linear, linestyle='--',
            linewidth=2.0,
            color='blue',label=r'$u_r[v]$',zorder = 3)
    ax.plot(T, h_linear, linestyle='--',
            linewidth=2.0,
            color='red',label=r'$u_h[v]$',zorder = 4)
    ax.plot(T, m_linear, linestyle='-',
            linewidth=2.0,
            color='black',label=r'$u[v]$',zorder = 2)
    ax.legend(ncol=3,loc='upper left',borderpad=0.1, labelspacing=0.2, columnspacing= 0.5)
    ax.grid()
    ax.set_xlabel(r'$t(s)$')
    ax.set_ylabel(r'$v(m/s)$')
    ax.set_xlim(0, step*(len(A_control)))
    ax.set_ylim(-0.5, 1.1)
    #-------------------- plot w
    # step = 1.0
    # T = [t*step for t in range(len(A_control))]
    # fig = plt.figure(figsize=(10,3))
    # ax = fig.add_subplot(111)
    # ax.plot(T, c_angular, linestyle='--',
    #         linewidth=2.0,
    #         color='blue',label=r'$u_r[\omega]$',zorder = 3)
    # ax.plot(T, h_angular, linestyle='--',
    #         linewidth=2.0,
    #         color='red',label=r'$u_h[\omega]$',zorder = 4)
    # ax.plot(T, m_angular, linestyle='-',
    #         linewidth=2.0,
    #         color='black',label=r'$u[\omega]$',zorder = 2)
    # ax.legend(ncol=3,loc='upper left',borderpad=0.1, labelspacing=0.2, columnspacing= 0.5)
    # ax.grid()
    # ax.set_xlabel(r'$t(s)$')
    # ax.set_ylabel(r'$\omega(rad/s)$')
    # ax.set_xlim(0, step*(len(A_control)))
    # ax.set_ylim(-1.1, 1.5)
    #------------------------------ zoom in v
    # step = 1.0
    # T = [t*step for t in range(len(A_control))]
    # k1 = 710
    # k2 = 910
    # k3 = 1200
    # k4 = 1400
    # fig = plt.figure(figsize=(10,3))
    # ax1 = fig.add_subplot(121)
    # ax1.plot(T[k1:k2], c_linear[k1:k2], linestyle='--',
    #         linewidth=2.0,
    #         color='blue',label=r'$u_r[v]$',zorder = 3)
    # ax1.plot(T[k1:k2], h_linear[k1:k2], linestyle='--',
    #         linewidth=2.0,
    #         color='red',label=r'$u_h[v]$',zorder = 4)
    # ax1.plot(T[k1:k2], m_linear[k1:k2], linestyle='-',
    #         linewidth=2.0,
    #         color='black',label=r'$u[v]$',zorder = 2)
    # ax1.legend(ncol=3,loc='upper left',borderpad=0.1, labelspacing=0.2, columnspacing= 0.5)
    # ax1.grid()
    # ax1.set_xlabel(r'$t(s)$')
    # ax1.set_ylabel(r'$v(m/s)$')
    # ax1.set_xlim(k1*step, k2*step)
    # ax1.set_ylim(-0.5, 1.1)
    # ax2 = fig.add_subplot(122)
    # ax2.plot(T[k3:k4], c_linear[k3:k4], linestyle='--',
    #         linewidth=2.0,
    #         color='blue',label=r'$u_r[v]$',zorder = 3)
    # ax2.plot(T[k3:k4], h_linear[k3:k4], linestyle='--',
    #         linewidth=2.0,
    #         color='red',label=r'$u_h[v]$',zorder = 4)
    # ax2.plot(T[k3:k4], m_linear[k3:k4], linestyle='-',
    #         linewidth=2.0,
    #         color='black',label=r'$u[v]$',zorder = 2)
    # ax2.legend(ncol=3,loc='upper left',borderpad=0.1, labelspacing=0.2, columnspacing= 0.5)
    # ax2.grid()
    # ax2.set_xlabel(r'$t(s)$')
    # ax2.set_ylabel(r'$v(m/s)$')
    # ax2.set_xlim(k3*step, k4*step)
    # ax2.set_ylim(-0.5, 1.1)
    # ------------------------------ zoom in w
    # step = 1.0
    # T = [t*step for t in range(len(A_control))]
    # k1 = 710
    # k2 = 910
    # k3 = 1200
    # k4 = 1400
    # fig = plt.figure(figsize=(10,3))
    # ax1 = fig.add_subplot(121)
    # ax1.plot(T[k1:k2], c_angular[k1:k2], linestyle='--',
    #         linewidth=2.0,
    #         color='blue',label=r'$u_r[\omega]$',zorder = 3)
    # ax1.plot(T[k1:k2], h_angular[k1:k2], linestyle='--',
    #         linewidth=2.0,
    #         color='red',label=r'$u_h[\omega]$',zorder = 4)
    # ax1.plot(T[k1:k2], m_angular[k1:k2], linestyle='-',
    #         linewidth=2.0,
    #         color='black',label=r'$u[\omega]$',zorder = 2)
    # ax1.legend(ncol=3,loc='upper left',borderpad=0.1, labelspacing=0.2, columnspacing= 0.5)
    # ax1.grid()
    # ax1.set_xlabel(r'$t(s)$')
    # ax1.set_ylabel(r'$\omega(rad/s)$')
    # ax1.set_xlim(k1*step, k2*step)
    # ax1.set_ylim(-1.1, 1.5)
    # ax2 = fig.add_subplot(122)
    # ax2.plot(T[k3:k4], c_angular[k3:k4], linestyle='--',
    #         linewidth=2.0,
    #         color='blue',label=r'$u_r[\omega]$',zorder = 3)
    # ax2.plot(T[k3:k4], h_angular[k3:k4], linestyle='--',
    #         linewidth=2.0,
    #         color='red',label=r'$u_h[\omega]$',zorder = 4)
    # ax2.plot(T[k3:k4], m_angular[k3:k4], linestyle='-',
    #         linewidth=2.0,
    #         color='black',label=r'$u[\omega]$',zorder = 2)
    # ax2.legend(ncol=3,loc='upper left',borderpad=0.1, labelspacing=0.2, columnspacing= 0.5)
    # ax2.grid()
    # ax2.set_xlabel(r'$t(s)$')
    # ax2.set_ylabel(r'$\omega(rad/s)$')
    # ax2.set_xlim(k3*step, k4*step)
    # ax2.set_ylim(-1.1, 1.5)        
    plt.savefig(r'sim_control_v_2.pdf', bbox_inches = 'tight')

def plot_beta(A_beta):
    fig = plt.figure(figsize=(10,5))
    ax = fig.add_subplot(111)
    ax.plot(range(len(A_beta)), A_beta, color='b',
            linestyle='-', linewidth=5, marker='o', mfc='r',
            fillstyle='full', markersize=6, zorder=2)
    ax.set_xlabel(r'$Iteration$')
    ax.set_ylabel(r'$\beta_k$')
    ax.set_xlim(0, len(A_beta))
    ax.set_ylim(0, 12)
    ax.grid()
    plt.savefig(r'sim_beta_2.pdf', bbox_inches = 'tight')    


if __name__ == "__main__":
    
    from argparse import ArgumentParser
    
    parser = ArgumentParser(description='simulations results')
    parser.add_argument('--model','-m', default = 'hotel', type = str,
                    help='Model : [hotel, hospital, office]')
    parser.add_argument('--robot','-r',default = "turtletbot",help = "robot model")
    parser.add_argument('--input','-i',default = "turtlebot_hotel.p",help = "input file")
    
    args = parser.parse_args()
    model = args.model
    robot_model = args.robot
    input_file = args.input
    
    output_name = robot_model + '_' + model + '_results.png'
    
    
    A_robot_pose, A_control, A_beta = pickle.load(open(input_file, 'rb'))
    # draw_map('map.png')
    plot_traj_init('map.png', A_robot_pose, A_control, output_name = output_name)
    #plot_control(A_control)
    #plot_beta(A_beta[0])
    # print A_beta[0]
