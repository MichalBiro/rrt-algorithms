# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import math

import numpy as np
import cv2 as cv
import time
import imageio

from src.rrt.rrt_q import RRT_Q
from src.search_space.search_space import SearchSpace
from src.robot_arm import FK,loc2glo,glo2loc,IK
from Object_visualization_q import RotatedRect, object_visualize, convert_rectangle, path_sampling, robot_visualization

# Record the start time
start_time = time.time()

# search space dimensions
q1_space = (0,180)  # degrees
q2_space = (0,360)  # degrees
#search space 2D [x,y]
x =(0,1050) #1050
y =(0,745) #745
XY_dimensions = (x,y)


X_dimensions = np.array([q1_space,q2_space])  # dimensions of Search Space - x,y,angle

# obstacles - for intesection
# obstacle = (450,350,300,500,0)
obstacle = (525, 115, 130, 230, 0)

# Moving Object - parameters
object_center = (750, 275)
object_width = 200
object_height = 100
object_angle = 180
object = (object_center[0], object_center[1],object_width, object_height, object_angle)

#start an goal - in 2D [xy]
# x_init = (800, 200, 0)  # starting location
# x_goal = (250, 300, 0)  # goal location
# # start an goal - in Q [q1,q2]
# x_init = glo2loc(x_init)
# x_goal = glo2loc(x_goal)
# x_q_init = IK(x_init)
# x_q_goal = IK(x_goal)
# x_q_init = (math.degrees(x_q_init[0]),math.degrees(x_q_init[1]))
# x_q_goal = (math.degrees(x_q_goal[0]),math.degrees(x_q_goal[1]))

x_q_init = (0,225)
x_q_goal = (180,135)

x_init = FK(x_q_init)
x_init = loc2glo(x_init)
angle_loc_zero = x_init[2] # local zero for gripping object in the angle
# x_q_init = (math.degrees(x_q_init[0]),math.degrees(x_q_init[1]))
# x_q_goal = (math.degrees(x_q_goal[0]),math.degrees(x_q_goal[1]))
print ("Goal:",x_q_goal)

Q = np.array([(3, 1)])  # length of tree edges
r = 1  # length of the smallest edge to check for intersection with obstacles
max_samples = 5000  # max number of samples to take before timing out
prc = 0.1  # probability of checking for a connection to goal

# create search space
X = SearchSpace(X_dimensions)

# create rrt_search
rrt = RRT_Q(X, Q, x_q_init, x_q_goal, max_samples, r, prc, object, obstacle, XY_dimensions)
path = rrt.rrt_search()


# Record the end time
end_time = time.time()
# Calculate the runtime
runtime = end_time - start_time
print("Program runtime:", runtime, "seconds")
print(path)

# # plot
# plot = Plot("rrt_3d")
# plot.plot_tree(X, rrt.trees)
# if path is not None:
#     plot.plot_path(X, path)
# #plot.plot_obstacles(X)
# plot.plot_start(X, x_init)
# plot.plot_goal(X, x_goal)
# plot.draw(auto_open=True)


# PLOT in OpenCV
# Create a white image
img = np.full((y[1]-y[0], x[1]-x[0], 3), 255, dtype=np.uint8)

# prekazka
rect_points = convert_rectangle(obstacle)
rect_color = (200, 255, 0)
cv.rectangle(img, (rect_points[0], rect_points[1]), (rect_points[2], rect_points[3]), rect_color, thickness=-1)

# Draw the rotated rectangle
frames = []  # create GIF

img2 = np.copy(img)
for pos_q in path_sampling(path):
    pos_xy = FK(pos_q)
    pos_xy = loc2glo(pos_xy)
    center = (pos_xy[0], pos_xy[1])
    angle = 360-(pos_xy[2]-angle_loc_zero+object_angle)
    obstacle1 = RotatedRect(obstacle[0], obstacle[1], obstacle[2], obstacle[3], obstacle[4])
    [rotated_pts, intersection] = object_visualize(center,object_width, object_height, angle, obstacle1)
    # Draw a rectangle
    cv.polylines(img, [rotated_pts], isClosed=True, color=(255, 0, 100), thickness=2)
    cv.polylines(img2, [rotated_pts], isClosed=True, color=(0, 0, 0), thickness=1)

    # draw intersection between 2 objects
    cv.polylines(img, [intersection], isClosed=True, color=(0, 0, 255), thickness=2)

    # draw robot arm
    robot_joints = robot_visualization(pos_q)
    color = (0, 0, 0 )  # black color
    thickness = 3
    cv.line(img,robot_joints[0], robot_joints[1], color, thickness)
    cv.line(img,robot_joints[1], robot_joints[2], color, thickness)

    if len(intersection) != 0:
        print("Collision !")

    # otocenie obrazka - zrkadlovo y
    img = cv.flip(img, 0)
    frames.append(img)  # GIF
    cv.imshow("image", img)
    cv.waitKey()

    # reset picture
    # img = np.full((y[1]-y[0], x[1]-x[0], 3), 255, dtype=np.uint8)
    # rect_points = convert_rectangle(obstacle)
    # rect_color = (200, 255, 0)
    # cv.rectangle(img, (rect_points[0], rect_points[1]), (rect_points[2], rect_points[3]), rect_color, thickness=-1)

    # keeping tracks
    img = np.copy(img2)

##path in q1,q2 rotations of robot arm joints
# print (path)
# path_q= []
# for pos in path:
#     local = glo2loc(pos)
#     pos_q = IK(local)
#     glob_pos_q = q_glob2q_robot(pos_q)
#     path_q.append(glob_pos_q)

# print ("Path in Q")
# print (path_q)

  # Save GIF
# with imageio.get_writer("RRT_Q.gif",mode="I") as writer:
#     for frame in frames:
#         print("Adding frame to GIF file")
#         rgb_frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
#         writer.append_data(rgb_frame)