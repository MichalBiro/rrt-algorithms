# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import numpy as np
import cv2 as cv
import imageio
import time

from src.rrt.rrt_star import RRTStar
from src.search_space.search_space import SearchSpace
from src.utilities.plotting import Plot
from src.robot_arm import IK,glo2loc,q_glob2q_robot
from Object_visualization import RotatedRect, object_visualize,convert_rectangle, path_sampling

# Record the start time
start_time = time.time()

# search space dimensions
x =1050
y =745
angle = 360

X_dimensions = np.array([(0, x), (0, y), (0,angle)])  # dimensions of Search Space - x,y,angle

# obstacles - for intesection
#obstacle = (400,5,100,100,0) #(400,175,300,400,0)
obstacle = (525, 115, 130, 230, 0) # real obstacle

#start an goal
x_init = (800,350,0)  # starting location
x_goal = (200,350,180)  # goal location

# Moving Object - parameters
object_center = (x_init[0],x_init[1])
object_width = 350
object_height = 630
object_angle = 0
object = (object_center[0], object_center[1],object_width, object_height, object_angle)


Q = np.array([(10,5)])  # length of tree edges
r = 1  # length of smallest edge to check for intersection with obstacles
max_samples = 3000  # max number of samples to take before timing out
rewire_count = 10  # optional, number of nearby branches to rewire
prc = 0.1  # probability of checking for a connection to goal

# create Search Space
X = SearchSpace(X_dimensions)

# create rrt_search
rrt = RRTStar(X, Q, x_init, x_goal, max_samples, r, prc, rewire_count,object, obstacle)
path = rrt.rrt_star()

# Record the end time
end_time = time.time()
# Calculate the runtime
runtime = end_time - start_time
print("Program runtime:", runtime, "seconds")

# # plot
# plot = Plot("rrt_star_3d")
# plot.plot_tree(X, rrt.trees)
# if path is not None:
#     plot.plot_path(X, path)
# plot.plot_obstacles(X, Obstacles)
# plot.plot_start(X, x_init)
# plot.plot_goal(X, x_goal)
# plot.draw(auto_open=True)

# PLOT in OpenCV
# Create a white image
img = np.full((y, x, 3), 255, dtype=np.uint8)

# prekazka
rect_points = convert_rectangle(obstacle)
rect_color = (200, 255, 0)
cv.rectangle(img, (rect_points[0], rect_points[1]), (rect_points[2], rect_points[3]), rect_color, thickness=-1)
# interpretacia pre prienik

# Draw the rotated rectangle
frames = []  # create GIF

img2 = np.copy(img)
for pos in path_sampling(path):

    center = (pos[0], pos[1])
    angle = 360-(pos[2]+object_angle)
    obstacle1 = RotatedRect(obstacle[0], obstacle[1], obstacle[2], obstacle[3], obstacle[4])
    [rotated_pts, intersection] = object_visualize(center,object_width,object_height, angle, obstacle1)
    # Draw a rectangle
    cv.polylines(img, [rotated_pts], isClosed=True, color=(255, 0, 100), thickness=2)
    cv.polylines(img2, [rotated_pts], isClosed=True, color=(0, 0, 0), thickness=1)

    # draw intersection between 2 objects
    cv.polylines(img, [intersection], isClosed=True, color=(0, 0, 255), thickness=2)

    if len(intersection) != 0:
        print("Collision !")

    # otocenie obrazka - zrkadlovo y
    img = cv.flip(img, 0)
    frames.append(img)  # GIF
    cv.imshow("image", img)
    cv.waitKey()

    # reset picture
    img = np.full((y, x, 3), 255, dtype=np.uint8)
    cv.rectangle(img, (300, 0), (600, 400), (200, 255, 0), thickness=-1)

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

# #Save GIF
# with imageio.get_writer("RRT_star.gif",mode="I") as writer:
#     for frame in frames:
#         print("Adding frame to GIF file")
#         rgb_frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
#         writer.append_data(rgb_frame)



