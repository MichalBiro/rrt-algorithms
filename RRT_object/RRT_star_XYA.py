# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import numpy as np
import cv2 as cv
import imageio
import time
import csv

from src.rrt.rrt_star import RRTStar
from src.search_space.search_space import SearchSpace
from src.utilities.plotting import Plot
from src.robot_arm import IK,glo2loc,q_glob2q_robot
from Object_visualization import RotatedRect, object_visualize,convert_rectangle, path_sampling

# create empty file for outputs
output_file = "data_files/XYA_star_output_data.csv"
output_path_file = 'data_files/XYA_star_output_data_path.csv'
gif_file = "Test.gif"
# Record the start time
glo_start_time = time.time()
def main():

    files_declaration()
    data = input_data_load()

    ID = 0 # for saving data
    for input in data[ID:]:
        #input = data[10439]
        ID = ID + 1
        [path,runtime,solution,obstacle,object,rrt,Obstacles,X, x_init, x_goal, x_search_space, y_search_space] = path_finding_algorithm(input)

        save_data(path,ID,runtime,solution,data)

        if solution == 0: continue

        #frames = path_visualize(path, object, obstacle,x_search_space, y_search_space)
        #save_gif(frames)
        #plot_SearchSpace(path, Obstacles, X, x_init, x_goal, rrt)
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
    x_search_space =1050
    y_search_space =745
    angle = 360

    X_dimensions = np.array([(0, x_search_space), (0, y_search_space), (0,angle)])  # dimensions of Search Space - x,y,angle

    # obstacles for Searchspace
    Obstacles = np.array([(460,0,0,590,230,angle)])


    # obstacles - for intesection
    obstacle = (525, 115, 130, 230, 0) # real obstacle

    # start and goal
    x_init = (input[1], input[2], input[5])  # starting location
    x_goal = (230, 372, 180)  # (100, 150, 0)  # goal location

    # Moving Object - parameters
    object_center = (x_init[0],x_init[1])
    object_width = input[3]
    object_height = input[4]
    object_angle = input[5]
    object = (object_center[0], object_center[1],object_width, object_height, object_angle)

    # obstacles for Searchspace
    if object_width <= object_height:
        x_offset = object_width
        y_offset = object_height
    else:
        x_offset = object_height
        y_offset = object_width
    Obstacles = np.array([(460-x_offset/2, 0, 0, 590+x_offset/2, 230+y_offset/2, angle),
                          (0,0,0,x_offset/2,y_search_space,angle),
                          (x_search_space-x_offset/2,0,0,x_search_space,y_search_space,angle),
                          (x_offset/2,745-y_offset/2,0,x_search_space-x_offset/2,y_search_space,angle),
                          (x_offset/2,0,0,460-x_offset/2,y_offset/2,angle),
                          (590+x_offset/2,0,0,x_search_space-x_offset/2,y_offset/2,angle)
                          ])

    Q = np.array([(10,5)])  # length of tree edges
    r = 1  # length of smallest edge to check for intersection with obstacles
    max_samples = 2000  # max number of samples to take before timing out
    rewire_count = 10  # optional, number of nearby branches to rewire
    prc = 0.1  # probability of checking for a connection to goal

    # create Search Space
    X = SearchSpace(X_dimensions, Obstacles)

    # create rrt_search
    rrt = RRTStar(X, Q, x_init, x_goal, max_samples, r, prc, rewire_count,object, obstacle)
    path = rrt.rrt_star()

    # Record the end time
    end_time = time.time()
    # Calculate the runtime
    runtime = round(end_time - start_time, 4)
    print("Program runtime:", runtime, "seconds")

    if path is None:
        path = []

    if len(path) == 0:
        solution = 0
    else:
        solution = 1

    return path, runtime, solution, obstacle, object, rrt, Obstacles, X, x_init, x_goal, x_search_space, y_search_space

def save_data(path,ID,runtime,solution,data):
    # OUTPUT - zapisanie logov

    output_data = [ID, solution, runtime]

    print(round((ID / len(data)) * 100, 2), "%")
    print("ID - ", ID, "   | sol =", solution)
    # Append new data to the existing CSV file
    with open(output_file, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([output_data[0]] + list(output_data[1:]))
    # print("New data has been appended to", output_file)

    with open(output_path_file, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        flattened_path = [round(item, 2) for sublist in path for item in sublist]
        flattened_path.insert(0, ID)  # add ID
        writer.writerow(flattened_path)
    # print("Data saved to data.csv")

    # STOP the time
    glo_end_time = time.time()
    print("Global time of the proces", round((glo_end_time - glo_start_time) / 60), "min ",
          round(glo_end_time - glo_start_time, 2) % 60, " sec")

    return solution
def path_visualize(path,object, obstacle, x_search_space, y_search_space):
    # PLOT in OpenCV
    # Create a white image
    img = np.full((y_search_space, x_search_space, 3), 255, dtype=np.uint8)

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
        angle = 360-pos[2]
        obstacle1 = RotatedRect(obstacle[0], obstacle[1], obstacle[2], obstacle[3], obstacle[4])
        [rotated_pts, intersection] = object_visualize(center,object[2], object[3], angle, obstacle1)
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
        img = np.full((y_search_space, x_search_space, 3), 255, dtype=np.uint8)
        cv.rectangle(img, (300, 0), (600, 400), (200, 255, 0), thickness=-1)

        # keeping tracks
        img = np.copy(img2)

    return frames

def save_gif(frames):
    #Save GIF
    with imageio.get_writer(gif_file,mode="I") as writer:
        for frame in frames:
            print("Adding frame to GIF file")
            rgb_frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            writer.append_data(rgb_frame)

def plot_SearchSpace(path, Obstacles, X, x_init, x_goal, rrt):
    plot = Plot("rrt_3d")
    plot.plot_tree(X, rrt.trees)
    if path is not None:
        plot.plot_path(X, path)
    plot.plot_obstacles(X, Obstacles)
    plot.plot_start(X, x_init)
    plot.plot_goal(X, x_goal)
    plot.draw(auto_open=True)

# main program
main()
