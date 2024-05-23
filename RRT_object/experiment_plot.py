# Michal Biro
# 15.4 2024
#
# cript for ploting results from experiment

import matplotlib.pyplot as plt
import csv
import numpy as np

# Load output data
file_path = "data_files2/result_2Q_diag.csv" # Define the file name
result_data = []
with open(file_path, 'r', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        # Convert each element in the row from string to integer
        row = [float(x) for x in row]
        # Append the row to the data list
        result_data.append(row)

print (result_data)

width = []
height = []
percentage = []
time = []
for row in result_data:
    # Extract the values at the 3rd and 4th positions
    w = row[0]
    h = row[1]
    p = row[5]
    t = row[6]


    # Append the values to the list
    width.append(int(w))
    height.append(int(h))
    percentage.append(p)
    time.append(t)

print(width)
print(height)
print(percentage)

# 2D PLOT
# Create the plot
plt.figure(figsize=(9, 7))
scatter = plt.scatter(width, height, c=percentage, cmap='RdYlGn', vmin=0, vmax=100, marker='s',s=200)

# Add colorbar
plt.colorbar(scatter, label='úspešnost [%]')

# Set labels and title
plt.xlabel('Šírka [mm]')
plt.ylabel('Výška [mm]')
#plt.title('3DOF_q')

# Show the plot
plt.show()

# ----------- plot for time --------------
# Create the plot
plt.figure(figsize=(9, 7))
scatter = plt.scatter(width, height, c=time, cmap='RdYlGn_r', vmin=0, vmax=20, marker='s',s=200)

# Add colorbar
plt.colorbar(scatter, label='čas [s]')

# Set labels and title
plt.xlabel('šírka [mm]')
plt.ylabel('výška [mm]')
#plt.title('3DOF_q - time')

# Show the plot
plt.show()

# # 3D PLOT
# # Create a new figure
# fig = plt.figure()
#
# # Add a 3D subplot
# ax = fig.add_subplot(111, projection='3d')
#
# # Define colormap
# cmap = plt.get_cmap('RdYlGn')
# # Normalize the data to [0, 1] for correct colormap mapping
# norm = plt.Normalize(min(percentage), max(percentage))
# # Define colors based on number of rectangles using a colormap
# colors = [cmap(norm(value)) for value in percentage]
#
# # Plot 3D bars
# for i in range(len(width)):
#     ax.bar3d(width[i], height[i], 0, 19, 19, percentage[i], color=colors[i])
#
# # Set labels for x, y, and z axes
# ax.set_xlabel('width')
# ax.set_ylabel('height')
# ax.set_zlabel('%')
#
# # Show the plot
# plt.show()

