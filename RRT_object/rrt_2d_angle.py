# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import numpy as np
import cv2 as cv
import shapely.geometry
import shapely.affinity
import imageio

from src.rrt.rrt import RRT
from src.search_space.search_space import SearchSpace
from src.utilities.plotting import Plot
from IK import IK,glo2loc,q_glob2q_robot
from Object_visualization import RotatedRect, object_visualize

# search space dimensions
x = 1000
y = 600
angle = 360

X_dimensions = np.array([(0, x), (0, y), (0,angle)])  # dimensions of Search Space - x,y,angle
# obstacles - for OpenCv
obstacle = (450,200,300,400,0)

# Moving Object - parameters 
center = (900, 250)
width = 20
height = 100
angle = 0
object = (center[0],center[1],width,height,angle)

#Obstacles = np.array([(200, 300, 300, 600),(700, 0, 800, 400)])
x_init = (900,250,0)  # starting location
x_goal = (100, 350,0)  # goal location

Q = np.array([(10, 5)])  # length of tree edges
r = 1  # length of smallest edge to check for intersection with obstacles
max_samples = 5000  # max number of samples to take before timing out
prc = 0.0  # probability of checking for a connection to goal

# create search space
X = SearchSpace(X_dimensions)

# create rrt_search
rrt = RRT(X, Q, x_init, x_goal, max_samples, r, prc, object, obstacle)
path = rrt.rrt_search(object, obstacle)


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
img = np.full((y, x, 3), 255, dtype=np.uint8)

cv.rectangle(img,(300,0),(600,400),(200,255,0),thickness=-1)
#interpretacia pre prienik
 
# Draw the rotated rectangle

# create GIF
frames = []
img2 = np.copy(img)

for pos in path:

    center = (pos[0],pos[1])
    angle = pos[2]
    obstacle1 = RotatedRect(obstacle[0],obstacle[1],obstacle[2],obstacle[3],obstacle[4])
    [rotated_pts, intersection] = object_visualize(center, width, height, angle, obstacle1)
    # Draw a rectangle
    cv.polylines(img, [rotated_pts], isClosed=True, color=(255, 0, 100), thickness=2)
    cv.polylines(img2, [rotated_pts], isClosed=True, color=(0, 0, 0), thickness=1)
  
    # draw intersection between 2 objects
    cv.polylines(img, [intersection], isClosed=True, color=(0, 0, 255), thickness=2)

    if len(intersection) != 0:
        print ("Collision !!!")

    frames.append(img) # GIF
    cv.imshow("image",img)
    cv.waitKey() 

    #reset picture
    img = np.full((y, x, 3), 255, dtype=np.uint8)
    cv.rectangle(img,(300,0),(600,400),(200,255,0),thickness=-1)

    #keeping tracks 
    img = np.copy(img2)


# print (path)
# path_q= []
# for pos in path:
#     local = glo2loc(pos)
#     pos_q = IK(local)
#     glob_pos_q = q_glob2q_robot(pos_q)
#     path_q.append(glob_pos_q) 

# print ("Path in Q")
# print (path_q)
    
# #Save GIF
# with imageio.get_writer("moving-object.gif",mode="I") as writer:
#     for frame in frames:
#         print("Adding frame to GIF file")
#         rgb_frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
#         writer.append_data(rgb_frame)