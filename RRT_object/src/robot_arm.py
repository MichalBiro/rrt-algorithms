# Michal Biro
# 27.1 2023
#
# scrip for getting joint rotations from x,y position

import math
import numpy as np
import sys


loc_zero=[525,(85+230),0]
# x=500
# y=185
# pos =[x,y]

#link dimension (in mm)
l1 = 360
l2 = 260

# global to local 
def glo2loc (glo_pos):
    loc_pos = [0,0,0]
    loc_pos[0]=glo_pos[0]-loc_zero[0]
    loc_pos[1]=glo_pos[1]-loc_zero[1]
    loc_pos[2]=glo_pos[2]-loc_zero[2]
    return loc_pos

def loc2glo (loc_pos):
    glo_pos = [0, 0, 0]
    glo_pos[0] = loc_pos[0] + loc_zero[0]
    glo_pos[1] = loc_pos[1] + loc_zero[1]
    glo_pos[2] = loc_pos[2] + loc_zero[2]
    return glo_pos


def q_glob2q_robot (q_pos):
    q1 = q_pos[0] + 1.57
    q2 = q_pos[1]
    q_pos_robot = [q1,q2]
    #q1 = 5

    # if (q1>3.15) or q1<0:    
    #     sys.exit("Error: Joint_1 out of range.")
    
    # if (q2>4.72) or q2<1.58:
    #     sys.exit("Error: Joint_2 out of range.")

    return q_pos_robot


#forward kinematics
def FK (q):
    q = (math.radians(q[0]), math.radians(q[1]))
    q1 = q[0]
    q2 = q[1]
    #angle = q[2]

    x1 = l1 * math.cos(q1)
    y1 = l1 * math.sin(q1)

    x2 = l2 * math.cos(q1+q2)
    y2 = l2 * math.sin(q1+q2)

    x = x1 + x2
    y = y1 + y2
    angle = q1+q2#+angle
    angle = math.degrees(angle)
    pos = (x,y,angle)
    return pos


#inverse kinemaics
def IK (pos):

    x=pos[0]
    y=pos[1]
    angle = pos[2]

    D = (x**2 + y**2 - l1**2 -l2**2)/(2*l1*l2)

    q2 = math.atan2(math.sqrt(1-D**2),D)
    gama = math.atan2(y,x)
    beta = math.atan2(l2*math.sin(q2),l1+l2*math.cos(q2))

    q1 = gama - beta

    q = (q1,q2,angle)

    return q

def diagonal(w,h):
    d = math.sqrt(w**2+h**2)
    alfa = math.atan2((w/2),(h/2))
    alfa = math.degrees(alfa)
    return d, alfa

def point_in_circle(point,center):

    radius = 445
    distance = math.sqrt((point[0] - center[0]) ** 2 + (point[1] - center[1]) ** 2)
    if distance < radius:
        return True
    else:
        return False


# p = (0,444)
# c = (0,0)
# x = point_in_circle(p,c)
# print (x)
#
# [d,alfa]=diagonal(350,100)
# print(d,alfa)
# # start an goal - in 2D [xy]
# x_init = (600, 100, 0)  # starting location
# x_goal = (100, 300, 0)  # goal location
# # start an goal - in Q [q1,q2]
# x_q_init = IK(x_init)
# x_q_goal = IK(x_goal)
# # x_q_init = (math.degrees(x_q_init[0]),math.degrees(x_q_init[1]))
# # x_q_goal = (math.degrees(x_q_goal[0]),math.degrees(x_q_goal[1]))
#
# q_i = (x_q_init)
# q_g = (x_q_goal)
# init = FK(q_i)
# goal = FK(q_g)
#
# print('hello')
# local=glo2loc(pos)
# print(local)
# print(IK(local))
# print(q_glob2q_robot(IK(local)))

# pos = (1.57,1.57)
# fk = FK(pos)
# print (fk)
