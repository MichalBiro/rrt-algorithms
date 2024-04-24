# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import math

import numpy as np
import cv2 as cv
import time
import csv
import imageio

from src.rrt.rrt_3q import RRT_Q
from src.search_space.search_space import SearchSpace
from src.utilities.plotting import Plot
from src.robot_arm import FK,loc2glo,IK
from Object_visualization_3q import RotatedRect, object_visualize, convert_rectangle, path_sampling, robot_visualization

# create empty file for outputs
output_file = "data_files/3DOF_Q_output_data-new.csv"
output_path_file='data_files/3DOF_Q_output_data_path-new.csv'
gif_file = "Test.gif"
# Record the start time
glo_start_time = time.time()

def main():

    #files_declaration()
    data = input_data_load()

    ID = 10735  # for saving data
    for input in data[ID:]:
        #input = data[8170]
        print(input)
        ID = ID + 1
        [path,runtime,solution,obstacle,object,rrt,Obstacles,X, x_q_init, x_q_goal, x_search_space, y_search_space, angle_loc_zero] = path_finding_algorithm(input)

        save_data(path, ID, runtime, solution, data)

        if solution == 0: continue

        #frames = path_visualize(path, object, obstacle, x_search_space, y_search_space, angle_loc_zero)
        #save_gif(frames)
        #plot_SearchSpace(path, Obstacles, X, x_q_init, x_q_goal, rrt)
def files_declaration():
    # # Open the file in write mode to create a new empty file
    with open(output_file, 'w', newline='') as csvfile:
        pass
    with open(output_path_file, 'w', newline='') as csvfile:
        pass

def input_data_load():
    # Load input data
    file_path = "data_files/input.csv"# Define the file name
    data = []
    with open(file_path, 'r', newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            # Convert each element in the row from string to integer
            row = [int(x) for x in row]
            # Append the row to the data list
            data.append(row)
    return data

def path_finding_algorithm(input):
    # Record the start time
    start_time = time.time()

    # search space dimensions
    q1_space = (0,180)  # degrees
    q2_space = (0,360)  # degrees
    q3_space = (0,360)  # degrees
    #search space 2D [x,y]
    x_search_space = 1050
    y_search_space = 745  # 600
    XY_dimensions = ((0,x_search_space),(0,y_search_space))

    X_dimensions = np.array([q1_space,q2_space,q3_space])  # dimensions of Search Space - x,y,angle

    # obstacles - for intesection
    obstacle = (525, 115, 130, 230, 0)

    # Moving Object - parameters
    object_center = (0, 0)
    object_width = input[3]
    object_height = input[4]
    object_angle = input[5]
    object = (object_center[0], object_center[1],object_width, object_height, object_angle)

    pos = (input[1],input[2],input[5])
    [q_up, q_down] = IK(pos)
    q_down = [math.degrees(q_down[0]), math.degrees(q_down[1])]
    x_q_init = (q_down[0], q_down[1],180)

    x_init = FK(x_q_init)
    x_init = loc2glo(x_init)
    angle_loc_zero = x_init[2] # local zero for gripping object in the angle
    x_q12_goal = (123, 125) # position of q1 and q2
    object_final_orientation = 180
    x_goal = FK(x_q12_goal)
    q3_rot = object_final_orientation-(x_goal[2]-angle_loc_zero-(180-object_angle)) #posiiono q3 to which should be rotated if the object has to be in the angle set in object_final_orientation
    # if q3_rot > 360:
    #     q3_rot = x_q_init[2] - (360-q3_rot)
    x_q_goal = (x_q12_goal[0],x_q12_goal[1],q3_rot)

    Q = np.array([(8, 5, 3, 1)])  # length of tree edges
    r = 1  # length of the smallest edge to check for intersection with obstacles
    max_samples = 2000  # max number of samples to take before timing out
    prc = 0.1  # probability of checking for a connection to goal

    # create search space
    X = SearchSpace(X_dimensions)

    # create rrt_search
    rrt = RRT_Q(X, Q, x_q_init, x_q_goal, max_samples, r, prc, object, obstacle, XY_dimensions)
    [path,Obstacles] = rrt.rrt_search()

    # Record the end time
    end_time = time.time()
    # Calculate the runtime
    runtime = round(end_time - start_time,4)
    print("Program runtime:", runtime, "seconds")
    print(path)

    if path is None:
        path = []

    if len(path) == 0:
        solution = 0
    else:
        solution = 1


    return path,runtime,solution,obstacle,object,rrt,Obstacles,X, x_q_init, x_q_goal, x_search_space, y_search_space, angle_loc_zero

def save_data(path, ID, runtime, solution, data):
    # OUTPUT - zapisanie logov

    output_data = [ID,solution, runtime]

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

    return solution

def path_visualize(path, object, obstacle, x_search_space, y_search_space, angle_loc_zero):
    # PLOT in OpenCV
    # Create a white image
    img = np.full((y_search_space, x_search_space, 3), 255, dtype=np.uint8)

    # prekazka
    rect_points = convert_rectangle(obstacle)
    rect_color = (200, 255, 0)
    cv.rectangle(img, (rect_points[0], rect_points[1]), (rect_points[2], rect_points[3]), rect_color, thickness=-1)

    # Draw the rotated rectangle
    frames = []  # create GIF

    img2 = np.copy(img)
    object_angle = object[4]
    for pos_q in path_sampling(path):
        pos_xy = FK(pos_q)
        pos_xy = loc2glo(pos_xy)
        center = (pos_xy[0], pos_xy[1])
        angle = 360-(pos_q[2] - angle_loc_zero + pos_xy[2] + object_angle)
        obstacle1 = RotatedRect(obstacle[0], obstacle[1], obstacle[2], obstacle[3], obstacle[4])
        [rotated_pts, intersection] = object_visualize(center,object[2], object[3], angle, obstacle1)
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
        img = np.full((y_search_space, x_search_space, 3), 255, dtype=np.uint8)
        rect_points = convert_rectangle(obstacle)
        rect_color = (200, 255, 0)
        cv.rectangle(img, (rect_points[0], rect_points[1]), (rect_points[2], rect_points[3]), rect_color, thickness=-1)

        # keeping tracks
        img = np.copy(img2)

    return frames

def save_gif(frames):
    # Save GIF
    with imageio.get_writer(gif_file, mode="I") as writer:
        for frame in frames:
            print("Adding frame to GIF file")
            rgb_frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            writer.append_data(rgb_frame)

def plot_SearchSpace(path, Obstacles, X, x_init, x_goal, rrt):
    plot = Plot("rrt_2d")
    plot.plot_tree(X, rrt.trees)
    if path is not None:
        plot.plot_path(X, path)
    plot.plot_obstacles(X, Obstacles)
    plot.plot_start(X, x_init)
    plot.plot_goal(X, x_goal)
    plot.draw(auto_open=True)

# main program
main()