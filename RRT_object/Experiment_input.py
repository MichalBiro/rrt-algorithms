# Micha Biro
# 8.4 2024
#
# script for generating inputs for experiment
# inputs - size of box, possition, rotation

# generate size
    # find 9 possitions based on size (sampling space)
        # try rotations - 0-180 step 10 deg
        # check if possible (colliosion)
        # chceck for same configuration
        # write to file - size, poss, rot

# repeat until the biggest object is generated

import csv
import math
from src.robot_arm import IK,FK,loc2glo
from Object_visualization import RotatedRect, object_visualize

def collision_check(center,size,angle):
    # funkcia na overenie kolizie objektu s prekazkou
    # Searchspace
    x_dimensions = (590, 1050)  # 1050
    y_dimensions = (0, 745)  # 745

    width = size[0]
    height = size[1]
    angle = 360 - (angle)
    obstacle1 = RotatedRect(525, 115, 130, 230, 0)
    [rotated_pts, intersection] = object_visualize(center, width, height, angle, obstacle1)
    # collision
    if len(intersection) != 0:
        print("Collision !")
        return True
    # out of bounds
    for points in rotated_pts:
        if points[0] < x_dimensions[0] or points[0] > x_dimensions[1] or points[1] < y_dimensions[0] or points[1] > y_dimensions[1]:
            print("Out of searchspace !")
            return True

    return False

# pos = [850,600,0]
# [q_up,q_down] = IK(pos)
# q_up = [math.degrees(q_up[0]),math.degrees(q_up[1])]
# q_down = [math.degrees(q_down[0]),math.degrees(q_down[1])]
# print(q_up)
# print(q_down)
# pos = FK(q_down)
# pos = loc2glo(pos)
# print(pos)

# # Define your 2D array
# inputs = [
#     [650, 200, 100, 40, 180],
#     [800, 300, 100, 200, 120],
#     [750, 500, 150, 150, 200],
#     [800, 600, 50, 200, 60]
# ]


inputs = []
ID = 0
x_size_range = [100,400]
y_size_range = [200,600]
increment = 20

x_size = x_size_range[0]
y_size = y_size_range[0]

while x_size <= x_size_range[1]:

    while y_size <= y_size_range[1]:


        # checks and eliminate unwanted cases
        diagonal = math.sqrt(x_size ** 2 + y_size ** 2)
        if x_size > y_size or diagonal < 370:
            y_size += increment
            continue

        object_size = [x_size,y_size]
        angle_base = 180
        rot_sampling = 20
        box_zero = [590,0]
        box_size = [460,745]
        box_offset = 50

        center_space = [box_size[0]-object_size[0] - 2*box_offset,
                        box_size[1]-object_size[1] - 2*box_offset]

        for i in range (3):
            for j in range (3):
                for rot in range (rot_sampling-1):
                    angle = angle_base + int(rot*(180/rot_sampling)-90)
                    center_x = box_zero[0] + box_offset + object_size[0] / 2 + j * (center_space[0] / 2)
                    center_y = box_zero[1] + box_offset + object_size[1] / 2 + i * (center_space[1] / 2)
                    center = (center_x,center_y)
                    poss_in_box = (i*3)+(j+1)
                    if collision_check(center,object_size,angle) == True:
                        print(angle)
                        continue
                    ID = ID + 1
                    inputs.append([ID,int(center_x),int(center_y),object_size[0],object_size[1],angle,poss_in_box])

        y_size += increment
    x_size += increment
    y_size = y_size_range[0]
    print (x_size)



print(inputs)
# Define the file name
file_name = "input2.csv"

# Write the array to the CSV file
with open(file_name, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerows(inputs)

print(f"Data has been written to {file_name}")



# # Initialize an empty list to store the read data
# data = []
#
# # Read data from the CSV file
# with open("output_data.csv", 'r', newline='') as csvfile:
#     reader = csv.reader(csvfile)
#     for row in reader:
#         # Convert each element in the row from string to integer
#         row = [int(x) for x in row]
#         # Append the row to the data list
#         data.append(row)
#
# print (data)
# print (data[1][1])



# # Define the variable
# data = [(10, 15), (20, 48)]
#
# # Save to a CSV file
# with open('data.csv', 'w', newline='') as csvfile:
#     writer = csv.writer(csvfile)
#     writer.writerows(data)
#
# print("Data saved to data.csv")
#
# # Read from the CSV file
# with open('output_data_path.csv', 'r') as csvfile:
#     reader = csv.reader(csvfile)
#     read_data = [[int(value) for value in row] for row in reader]
#
# print("Data read from data.csv:", read_data)
#
# print(read_data[0])

