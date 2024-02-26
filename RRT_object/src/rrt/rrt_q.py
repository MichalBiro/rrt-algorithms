from src.rrt.rrt_q_base import RRTBase_Q
import matplotlib.pyplot as plt
import imageio
import numpy as np


class RRT_Q(RRTBase_Q):
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
        super().__init__(X, Q, x_init, x_goal, max_samples, r, prc, object, obstacle, XY_dimensions)

    def rrt_search(self):
        """
        Create and return a Rapidly-exploring Random Tree, keeps expanding until can connect to goal
        https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
        :return: list representation of path, dict representing edges of tree in form E[child] = parent
        """
        self.add_vertex(0, self.x_init)
        self.add_edge(0, self.x_init, None)

        while True:
            for q in self.Q:  # iterate over different edge lengths until solution found or time out
                for i in range(q[1]):  # iterate over number of edges of given length to add
                    x_new, x_nearest = self.new_and_near(0, q)

                    if x_new is None:
                        continue

                    # connect shortest valid edge
                    self.connect_to_point(0, x_nearest, x_new)

                    solution = self.check_solution()
                    if solution[0]:
                        return solution[1]

