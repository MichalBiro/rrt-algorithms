# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import math

import numpy as np
import cv2 as cv
import time
import csv
import imageio

from src.rrt.rrt_q import RRT_Q
from src.search_space.search_space import SearchSpace
from src.robot_arm import FK,loc2glo,glo2loc,IK
from Object_visualization_q import RotatedRect, object_visualize, convert_rectangle, path_sampling, robot_visualization

# create empty file for outputs
output_file = "output_data2.csv"
output_path_file='output_data_path2.csv'
# # Open the file in write mode to create a new empty file
with open(output_file, 'w', newline='') as csvfile:
    pass
with open(output_path_file, 'w', newline='') as csvfile:
    pass

# Record the start time
    glo_start_time = time.time()
# Load input data
# Read data from the CSV file
# Define the file name
file_name = "input.csv"
data = []
with open(file_name, 'r', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        # Convert each element in the row from string to integer
        row = [int(x) for x in row]
        # Append the row to the data list
        data.append(row)

ID = 0 #for saving data
for input in data:
    #input = data[1827]
    ID = ID + 1
    pos = (input[1],input[2],input[5])
    [q_up, q_down] = IK(pos)
    q_down = [math.degrees(q_down[0]), math.degrees(q_down[1])]

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
    obstacle = (525, 115, 130, 230, 0)

    # Moving Object - parameters
    object_center = (0, 0)
    object_width = input[3]
    object_height = input[4]
    object_angle = input[5]
    object = (object_center[0], object_center[1],object_width, object_height, object_angle)

    #x_q_init = (input[0],input[1])
    x_q_init = (q_down[0], q_down[1])
    x_q_goal = (115,130)

    x_init = FK(x_q_init)
    x_init = loc2glo(x_init)
    angle_loc_zero = x_init[2] # local zero for gripping object in the angle
    # x_q_init = (math.degrees(x_q_init[0]),math.degrees(x_q_init[1]))
    # x_q_goal = (math.degrees(x_q_goal[0]),math.degrees(x_q_goal[1]))
    #print ("Goal:",x_q_goal)

    Q = np.array([(3, 1)])  # length of tree edges
    r = 1  # length of the smallest edge to check for intersection with obstacles
    max_samples = 2000  # max number of samples to take before timing out
    prc = 0.1  # probability of checking for a connection to goal

    # create search space
    X = SearchSpace(X_dimensions)

    # create rrt_search
    rrt = RRT_Q(X, Q, x_q_init, x_q_goal, max_samples, r, prc, object, obstacle, XY_dimensions)
    [path,pre_rot] = rrt.rrt_search()

    # Record the end time
    end_time = time.time()
    # Calculate the runtime
    runtime = round(end_time - start_time,4)
    print("Program runtime:", runtime, "seconds")
    #print(path)


    #----------------------------------------------------------------------
    # OUTPUT - zapisanie logov
    if path is None:
        path = []

    if len(path) == 0:
        solution = 0
        output_data = [ID,solution, runtime, 0]
    else:
        solution = 1
        output_data = [ID,solution, runtime, abs(object_angle - pre_rot)]

    print (round((ID/len(data))*100,2),"%")
    print ("ID - ",ID,"   | sol =",solution)
    # Append new data to the existing CSV file
    with open(output_file, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([output_data[0]] + list(output_data[1:]))

    #print("New data has been appended to", output_file)

    with open(output_path_file, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        flattened_path = [round(item,2) for sublist in path for item in sublist]
        flattened_path.insert(0, ID) # add ID
        writer.writerow(flattened_path)

    #print("Data saved to data.csv")

    # STOP the time
    glo_end_time = time.time()
    print ("Global time of the proces",round((glo_end_time - glo_start_time)/60),"min ",(round(glo_end_time - glo_start_time,2)) % 60," sec")

    if solution == 0: continue
    # #----------------------------------------------------------------------
    # # PLOT in OpenCV
    # # Create a white image
    # img = np.full((y[1]-y[0], x[1]-x[0], 3), 255, dtype=np.uint8)
    #
    # # prekazka
    # rect_points = convert_rectangle(obstacle)
    # rect_color = (200, 255, 0)
    # cv.rectangle(img, (rect_points[0], rect_points[1]), (rect_points[2], rect_points[3]), rect_color, thickness=-1)
    #
    # # Draw the rotated rectangle
    # frames = []  # create GIF
    #
    # img2 = np.copy(img)
    # # prerotation of object
    # diff = abs(object_angle-pre_rot)
    # rot = 0
    # while rot < diff:
    #     pos_xy = FK(path[0])
    #     pos_xy = loc2glo(pos_xy)
    #     center = (pos_xy[0], pos_xy[1])
    #     if pre_rot > object_angle: angle = 360 - (pos_xy[2] - angle_loc_zero + object_angle + rot)
    #     else: angle = 360 - (pos_xy[2] - angle_loc_zero + object_angle - rot)
    #     rot = rot+2
    #     obstacle1 = RotatedRect(obstacle[0], obstacle[1], obstacle[2], obstacle[3], obstacle[4])
    #     [rotated_pts, intersection] = object_visualize(center, object_width, object_height, angle, obstacle1)
    #     # Draw a rectangle
    #     cv.polylines(img, [rotated_pts], isClosed=True, color=(255, 0, 100), thickness=2)
    #     cv.polylines(img2, [rotated_pts], isClosed=True, color=(0, 0, 0), thickness=1)
    #
    #     # draw intersection between 2 objects
    #     cv.polylines(img, [intersection], isClosed=True, color=(0, 0, 255), thickness=2)
    #
    #     if len(intersection) != 0:
    #         print("Collision !")
    #
    #     # otocenie obrazka - zrkadlovo y
    #     img = cv.flip(img, 0)
    #     frames.append(img)  # GIF
    #     cv.imshow("image", img)
    #     cv.waitKey()
    #     # # reset picture
    #     # img = np.full((y[1] - y[0], x[1] - x[0], 3), 255, dtype=np.uint8)
    #     # rect_points = convert_rectangle(obstacle)
    #     # rect_color = (200, 255, 0)
    #     # cv.rectangle(img, (rect_points[0], rect_points[1]), (rect_points[2], rect_points[3]), rect_color, thickness=-1)
    #     # keeping tracks
    #     img = np.copy(img2)
    #
    # object_angle = pre_rot
    # for pos_q in path_sampling(path):
    #     pos_xy = FK(pos_q)
    #     pos_xy = loc2glo(pos_xy)
    #     center = (pos_xy[0], pos_xy[1])
    #     angle = 360-(pos_xy[2]-angle_loc_zero+object_angle)
    #     obstacle1 = RotatedRect(obstacle[0], obstacle[1], obstacle[2], obstacle[3], obstacle[4])
    #     [rotated_pts, intersection] = object_visualize(center,object_width, object_height, angle, obstacle1)
    #     # Draw a rectangle
    #     cv.polylines(img, [rotated_pts], isClosed=True, color=(255, 0, 100), thickness=2)
    #     cv.polylines(img2, [rotated_pts], isClosed=True, color=(0, 0, 0), thickness=1)
    #
    #     # draw intersection between 2 objects
    #     cv.polylines(img, [intersection], isClosed=True, color=(0, 0, 255), thickness=2)
    #
    #     # draw robot arm
    #     robot_joints = robot_visualization(pos_q)
    #     color = (0, 0, 0 )  # black color
    #     thickness = 3
    #     cv.line(img,robot_joints[0], robot_joints[1], color, thickness)
    #     cv.line(img,robot_joints[1], robot_joints[2], color, thickness)
    #
    #     if len(intersection) != 0:
    #         print("Collision !")
    #
    #     # otocenie obrazka - zrkadlovo y
    #     img = cv.flip(img, 0)
    #     frames.append(img)  # GIF
    #     cv.imshow("image", img)
    #     cv.waitKey()
    #
    #     # reset picture
    #     img = np.full((y[1]-y[0], x[1]-x[0], 3), 255, dtype=np.uint8)
    #     rect_points = convert_rectangle(obstacle)
    #     rect_color = (200, 255, 0)
    #     cv.rectangle(img, (rect_points[0], rect_points[1]), (rect_points[2], rect_points[3]), rect_color, thickness=-1)
    #
    #     # keeping tracks
    #     img = np.copy(img2)
    # #------------------------------------------------------------------------------------------------------------------

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