#!/usr/bin/env python3
import roslib
import numpy
import queue
roslib.load_manifest('hil_mix_control')
import rospy
import sys

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist

from math import pi as PI
from math import atan2, sin, cos, sqrt, exp

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from init_sys_willowgarage import sys_model

def TaskCallback(taskdata):
    # MultiArrayLayout data
    global temp_task
    s_x = taskdata.sx
    s_y = taskdata.sy
    g_x = taskdata.gx
    g_y = taskdata.gy    
    tsg = taskdata.tsg
    temp_task = (s_x, s_y, g_x, g_y, tsg)
    print ('robot received temporary task <>(%s && <> %s)' %(str((s_x,s_y)), str((g_x,g_y))))
    return temp_task

def PoseCallback(posedata):
    # PoseWithCovarianceStamped data from amcl_pose
    global robot_pose # [time, [x,y,yaw]]
    header = posedata.header
    pose = posedata.pose
    if (not robot_pose[0]) or (header.stamp.secs > robot_pose[0]):
        # more recent pose data received
        robot_pose[0] = header.stamp.secs
        # TODO: maybe add covariance check here?
        # print('robot position update!')
        euler = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]) #roll, pitch, yaw
        robot_pose[1] = [pose.pose.position.x, pose.pose.position.y, euler[2]] # in radians
    return robot_pose

def NaviControlCallback(twistdata):
    global navi_control
    linear_v = twistdata.linear.x
    angular_v = twistdata.angular.z
    navi_control = [linear_v, angular_v]

def TeleControlCallback(twistdata):
    global tele_control
    linear_v = twistdata.linear.x
    angular_v = twistdata.angular.z
    tele_control = [linear_v, angular_v] 


def hil_planner(robot_name='turtlebot'):
    global robot_pose, navi_control, tele_control, temp_task
    # robot_full_model, hard_task, soft_task = sys_model
    robot_pose = [0, [0, 0, 0]]
    navi_control = [0, 0]
    tele_control = [0, 0]
    mix_control = [0, 0]
    
    #GoalPublisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size = 100)
    #MixPublisher = rospy.Publisher('cmd_vel', Twist, queue_size = 100)

    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, PoseCallback)
    # control command from amcl navigation
    rospy.Subscriber('nav_vel', Twist, NaviControlCallback)
    # control command from tele operation
    rospy.Subscriber('key_vel', Twist, TeleControlCallback)
    # temporary task
    rospy.Subscriber('temp_task', task, TaskCallback)

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        print('teleoperation {}'.format(tele_control))
        print('navigation {}'.format(navi_control))
        print('position {}'.format())

        r.sleep()

if __name__ == '__main__':
    hil_planner()