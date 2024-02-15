# Michal Biro
# 27.1 2023
#
# scrip for getting joint rotations from x,y position

import math
import numpy as np
import sys


loc_zero=[50,8.5]
x=50
y=18.5
pos =[x,y]

#link dimension (in cm)
l1 = 36
l2 = 26

# global to local 
def glo2loc (glo_pos):
    loc_pos = [0,0]
    loc_pos[0]=glo_pos[0]-loc_zero[0]
    loc_pos[1]=glo_pos[1]-loc_zero[1]

    return loc_pos

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


def IK (pos):

    x=pos[0]
    y=pos[1] 

    D = (x**2 + y**2 - l1**2 -l2**2)/(2*l1*l2)

    q2 = math.atan2(math.sqrt(1-D**2),D)
    gama = math.atan2(y,x)
    beta = math.atan2(l2*math.sin(q2),l1+l2*math.cos(q2))

    q1 = gama - beta

    q = [q1,q2]

    return q



# local=glo2loc(pos)
# print(local)
# print(IK(local))
# print(q_glob2q_robot(IK(local)))
