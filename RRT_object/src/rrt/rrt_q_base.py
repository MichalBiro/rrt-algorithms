import math
import random

import numpy as np
import math
import sys

from src.rrt.tree import Tree
from src.utilities.geometry import steer
from src.robot_arm import FK, glo2loc, loc2glo
from Object_visualization_q import RotatedRect, object_visualize


class RRTBase_Q(object):
    def __init__(self, X, Q, x_init, x_goal, max_samples, r, prc=0.01, object=None, obstacle=None, XY_dimensions=None):
        """
        Template RRT planner
        :param X: Search Space
        :param Q: list of lengths of edges added to tree
        :param x_init: tuple, initial location
        :param x_goal: tuple, goal location
        :param max_samples: max number of samples to take
        :param r: resolution of points to sample along edge when checking for collisions
        :param prc: probability of checking whether there is a solution
        """
        self.X = X
        self.samples_taken = 0
        self.max_samples = max_samples
        self.Q = Q
        self.r = r
        self.prc = prc
        self.x_init = x_init
        self.x_goal = x_goal
        self.object = object
        self.obstacle = obstacle
        self.XY_dimensions = XY_dimensions
        self.trees = []  # list of all trees
        self.add_tree()  # add initial tree

    def add_tree(self):
        """
        Create an empty tree and add to trees
        """
        self.trees.append(Tree(self.X))

    def add_vertex(self, tree, v):
        """
        Add vertex to corresponding tree
        :param tree: int, tree to which to add vertex
        :param v: tuple, vertex to add
        """
        self.trees[tree].V.insert(0, v + v, v)
        self.trees[tree].V_count += 1  # increment number of vertices in tree
        self.samples_taken += 1  # increment number of samples taken

    def add_edge(self, tree, child, parent):
        """
        Add edge to corresponding tree
        :param tree: int, tree to which to add vertex
        :param child: tuple, child vertex
        :param parent: tuple, parent vertex
        """
        self.trees[tree].E[child] = parent

    def nearby(self, tree, x, n):
        """
        Return nearby vertices
        :param tree: int, tree being searched
        :param x: tuple, vertex around which searching
        :param n: int, max number of neighbors to return
        :return: list of nearby vertices
        """
        return self.trees[tree].V.nearest(x, num_results=n, objects="raw")

    def get_nearest(self, tree, x):
        """
        Return vertex nearest to x
        :param tree: int, tree being searched
        :param x: tuple, vertex around which searching
        :return: tuple, nearest vertex to x
        """
        return next(self.nearby(tree, x, 1))

    def new_and_near(self, tree, q):
        """
        Return a new steered vertex and the vertex in tree that is nearest
        :param tree: int, tree being searched
        :param q: length of edge when steering
        :return: vertex, new steered vertex, vertex, nearest vertex in tree to new vertex
        """
        x_rand = self.X.sample_free()
        x_nearest = self.get_nearest(tree, x_rand)
        x_new = self.bound_point(steer(x_nearest, x_rand, q[0]))

        # check if new point is in X_free and not already in V
        if not self.trees[0].V.count(x_new) == 0 or self.collision_check(x_new) == True:
            return None, None


        self.samples_taken += 1
        return x_new, x_nearest

    def collision_check(self, x_new, pre_rot=0):
        #transformacia z uhlov na suradnice xy
        x_new = self.q2xya(x_new)
        x_init = self.q2xya(self.x_init)
        angle_loc_zero = x_init[2]

        # funkcia na overenie kolizie objektu s prekazkou
        center = (x_new[0], x_new[1])
        width = self.object[2]
        height = self.object[3]
        angle = 360-(x_new[2]-angle_loc_zero+self.object[4] + pre_rot)
        obstacle1 = RotatedRect(self.obstacle[0], self.obstacle[1], self.obstacle[2], self.obstacle[3],self.obstacle[4])
        [rotated_pts, intersection] = object_visualize(center, width, height, angle, obstacle1)
        # collision
        if len(intersection) != 0:
            #print("Collision !")
            return True
        # out of bounds
        for points in rotated_pts:
            if points[0] < self.XY_dimensions[0][0] or points[0] > self.XY_dimensions[0][1] or points[1] < self.XY_dimensions[1][0] or points[1] > self.XY_dimensions[1][1]:
                #print("Out of searchspace !")
                return True

        return False

    def connect_to_point(self, tree, x_a, x_b):
        """
        Connect vertex x_a in tree to vertex x_b
        :param tree: int, tree to which to add edge
        :param x_a: tuple, vertex
        :param x_b: tuple, vertex
        :return: bool, True if able to add edge, False if prohibited by an obstacle
        """

        if self.trees[tree].V.count(x_b) == 0: #and self.linear_sampling_collision_check(x_a,x_b) == False:
            self.add_vertex(tree, x_b)
            self.add_edge(tree, x_b, x_a)
            return True
        return False

    def can_connect_to_goal(self, tree):
        """
        Check if the goal can be connected to the graph
        :param tree: rtree of all Vertices
        :return: True if can be added, False otherwise
        """
        x_nearest = self.get_nearest(tree, self.x_goal)
        if self.x_goal in self.trees[tree].E and x_nearest in self.trees[tree].E[self.x_goal]:
            # tree is already connected to goal using nearest vertex
            return True
        if self.linear_sampling_collision_check(x_nearest,self.x_goal) == False:  # check if obstacle-free
            return True
        return False

    def get_path(self):
        """
        Return path through tree from start to goal
        :return: path if possible, None otherwise
        """
        if self.can_connect_to_goal(0):
            #print("Can connect to goal")
            #print("Samples taken:",self.samples_taken)
            self.connect_to_goal(0)
            return self.reconstruct_path(0, self.x_init, self.x_goal)
        #print("Could not connect to goal")
        return None

    def connect_to_goal(self, tree):
        """
        Connect x_goal to graph
        (does not check if this should be possible, for that use: can_connect_to_goal)
        :param tree: rtree of all Vertices
        """
        x_nearest = self.get_nearest(tree, self.x_goal)
        self.trees[tree].E[self.x_goal] = x_nearest

    def reconstruct_path(self, tree, x_init, x_goal):
        """
        Reconstruct path from start to goal
        :param tree: int, tree in which to find path
        :param x_init: tuple, starting vertex
        :param x_goal: tuple, ending vertex
        :return: sequence of vertices from start to goal
        """
        path = [x_goal]
        current = x_goal
        if x_init == x_goal:
            return path
        while not self.trees[tree].E[current] == x_init:
            path.append(self.trees[tree].E[current])
            current = self.trees[tree].E[current]
        path.append(x_init)
        path.reverse()
        return path

    def check_solution(self):
        # probabilistically check if solution found

        if (self.prc and random.random() < self.prc) or self.samples_taken <=1:
            #print("Checking if can connect to goal at", str(self.samples_taken), "samples")
            path = self.get_path()
            if path is not None:
                return True, path
        # check if can connect to goal after generating max_samples
        if self.samples_taken >= self.max_samples:
            return True, self.get_path()
        return False, None

    def bound_point(self, point):
        # if point is out-of-bounds, set to bound
        point = np.maximum(point, self.X.dimension_lengths[:, 0])
        point = np.minimum(point, self.X.dimension_lengths[:, 1])
        return tuple(point)

    def q2xya(self,q):
        xya = FK(q)  # from Q_space to XY_space
        xya = loc2glo(xya)
        return xya

    def point_in_circle(self,point,center):

        radius = 445 # distance from obstacle to first joint of the arm
        distance = math.sqrt((point[0] - center[0]) ** 2 + (point[1] - center[1]) ** 2)
        if distance < radius:
            return True
        else:
            return False

    def object_in_circle(self,object,angle):

        xya = self.q2xya((90,180))
        init_xya = self.q2xya(self.x_init)

        center = (xya[0],xya[1])
        angle = angle + (xya[2]-init_xya[2])
        obstacle1 = RotatedRect(self.obstacle[0], self.obstacle[1], self.obstacle[2], self.obstacle[3],self.obstacle[4])
        [rotated_pts, intersection] = object_visualize(center, object[2], object[3], angle, obstacle1)

        circle_center = (525,675) # y = 230(obstacle) + 85(base_link) + 360(link_1)
        for point in rotated_pts:
            if self.point_in_circle(point,circle_center)==False:
                return False
        return True


    def diagonal(self,h, w):
        d = math.sqrt(w ** 2 + h ** 2)
        alfa = math.atan2((w / 2), (h / 2))
        alfa = math.degrees(alfa)
        return d #, alfa

    def poss_prerot_init(self, pos_q ,object):
        pre_rot = []
        rot = 0
        increment = 1
        angle = object[4]
        # up from current angle
        while angle < 360:
            if self.collision_check(pos_q,rot) == True:
                break
            pre_rot.append(angle)
            rot = rot + increment
            angle = angle + increment


        rot = 0
        angle = object[4]-1
        while angle >= 0:
            if self.collision_check(pos_q, rot) == True:
                break
            pre_rot.append(angle)
            rot = rot - increment
            angle = angle - increment

        return pre_rot
    def poss_prerot_goal(self, pos_q ,object):
        pre_rot = []
        rot = 0
        increment = 1
        angle = object[4]
        # up from current angle
        while angle < 360:
            if self.collision_check(pos_q,rot) == False:
                pre_rot.append(angle)
            rot = rot + increment
            angle = angle + increment


        rot = 0
        angle = object[4]-1
        while angle >= 0:
            if self.collision_check(pos_q, rot) == False:
                pre_rot.append(angle)
            rot = rot - increment
            angle = angle - increment

        return pre_rot
    def linear_sampling_collision_check(self,start,goal):

        increment = 3  #degrees
        range_q1 = abs(start[0] - goal[0])
        range_q2 = abs(start[1] - goal[1])
        d = math.sqrt(range_q1 ** 2 + range_q2 ** 2)  # prepona
        samples = int(d / increment)
        if samples == 0:
            return False

        increment_x = range_q1 / samples
        increment_y = range_q2 / samples

        for i in range(1,samples):
            if start[0] < goal[0]: q1 = start[0] + i*increment_x
            else: q1 = start[0] - i*increment_x

            if start[1] < goal[1]: q2 = start[1] + i*increment_y
            else: q2 = start[1] - i*increment_y

            position = (q1,q2)
            if self.collision_check(position):
                return True

        return False

    def pre_rot_sampling_collision_check(self,angle):
        object_angle = self.object[4]
        diff = abs(object_angle - angle)
        rot = 0
        inc = 3
        while abs(rot) < diff:
            if angle > object_angle:
                rot = rot + inc
            else:
                rot = rot - inc
            if self.collision_check(self.x_init, rot):
                return False
        return True

    def pre_rot(self):
        final_pre_rot = []
        # check possible prerotations - init and goal position
        init_pre_rot = self.poss_prerot_init(self.x_init, self.object)
        goal_pre_rot = self.poss_prerot_goal(self.x_goal, self.object)
        init_pre_rot = set(init_pre_rot)
        goal_pre_rot = set(goal_pre_rot)
        # combine 2 sets
        possible_pre_rot = init_pre_rot.intersection(goal_pre_rot)
        possible_pre_rot = list(possible_pre_rot)
        min_diff = 360
        # iterate through poss_pre_rot and find the smallest possible rotation
        for angle in possible_pre_rot:
            if self.object_in_circle(self.object,angle):  # and self.pre_rot_sampling_collision_check(angle): # check if points of object are inside collision-free circle
                diff = abs(angle - self.object[4])
                if diff < min_diff:
                    final_pre_rot = angle
                    min_diff = diff
        #print("Pre rotation", final_pre_rot)
        #print(self.object)
        self.object = (self.object[0], self.object[1], self.object[2], self.object[3], final_pre_rot)
        #print(self.object)
        # if no solution found
        if not final_pre_rot:
            #print("No possible solution")
            #sys.exit()
            return

        return final_pre_rot

