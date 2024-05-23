# Michal Biro
# 14.4 2024
#
# script for evaluating outputs from experiment - reading files 2DOF and 3DOF_output_data
import csv
import math
import statistics

# Define the file name
result_file_path = "data_files2/result_XYA_star.csv" # results
data_file_path = "data_files2/XYA_star.csv" # data
datapath_file_path = "data_files2/XYA_star_path.csv" # path
file_path = "data_files2/input2.csv" # input

output_data = []
with open(data_file_path, 'r', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        # Convert each element in the row from string to integer
        row = [float(x) for x in row]
        # Append the row to the data list
        output_data.append(row)

output_path = []
with open(datapath_file_path, 'r', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        # Convert each element in the row from string to integer
        row = [float(x) for x in row]
        # Append the row to the data list
        output_path.append(row)

# Load input data
input_data = []
with open(file_path, 'r', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        # Convert each element in the row from string to integer
        row = [int(x) for x in row]
        # Append the row to the data list
        input_data.append(row)

object_size_data = []

for row in input_data: # [:7237]
    # Extract the values at the 3rd and 4th positions
    w = row[3]  # 3rd position (index 2)
    h = row[4]  # 4th position (index 3)

    # Append the values to the list
    object_size_data.append((w, h))

object_size_list = sorted(list(set(object_size_data)))

object_size_data = []
object_size_time = []
global_time = 0
for i,r_output in enumerate(output_data[:]):
    row = input_data[i]
    # Extract the values at the 3rd and 4th positions
    w = row[3]  # 3rd position (index 2)
    h = row[4]  # 4th position (index 3)
    sol = int(r_output[1])
    time = r_output[2]
    #print(i)

    # Append the values to the list
    object_size_data.append((w, h, sol))
    object_size_time.append((w, h, time))
    if time < 20: global_time += time
    else: global_time += 20

result=[]
global_success = 0
all_cases = 0

for w,h in object_size_list:
    success = object_size_data.count((w, h, 1))
    failure = object_size_data.count((w, h, 0))
    num_cases = success + failure
    if num_cases == 0:
        success_rate = 0
    else:
        success_rate = round((success/num_cases)*100,2)

    times = [sublist[2] for sublist in object_size_time if sublist[:2] == (w,h)]

    average_time = round(sum(times)/len(times),2)
    #print(w,h,success,failure,num_cases,success_rate)
    result.append([w,h,success,failure,num_cases,success_rate,average_time])
    global_success += success
    all_cases += num_cases
    global_average_time = round(global_time/all_cases,2)

err = 0
for w,h,time in object_size_time:
    e = (float(global_average_time)-time)**2
    err += e
standard_deviation = math.sqrt(err/all_cases)

print("Number of successful cases: ",global_success,"/",all_cases)
print("Success rate in all cases: ",(global_success/all_cases)*100)
print("Average time for finding path: ", global_average_time)
print("Standard deviation for time: ", standard_deviation)

# trajectory path
# shortest / actual
paths_ratios=[]
def distance(pos1,pos2):
    # for XYA and XYA_star
    x = abs(pos2[0] - pos1[0])
    y = abs(pos2[1] - pos1[1])
    a = abs(pos2[2] - pos1[2])

    d = math.sqrt(x**2 + y**2)
    distance = d+a

    # # for 3Q
    # q1 = abs(pos2[0] - pos1[0])
    # q2 = abs(pos2[1] - pos1[1])
    # q3 = abs(pos2[2] - pos1[2])
    #
    # distance = q1 + q2 + q3

    # # for 2Q
    # q1 = abs(pos2[0] - pos1[0])
    # q2 = abs(pos2[1] - pos1[1])
    #
    # distance = q1 + q2

    return distance

for path in output_path: #test_path:
    num_elements = 3  # variable for XYA,3Q - 3  2Q -2
    if len(path) < 2:
        continue

    path = path[1:] # cut the first element

    path_length = int(len(path)/num_elements)

    # shortest path
    start = path[0:num_elements]
    end = path[path_length*num_elements-num_elements:path_length*num_elements]
    best_path = distance(start,end)

    D = 0
    for i in range(path_length-1):
        pos1=path[i*num_elements:i*num_elements+num_elements]
        pos2=path[(i+1)*num_elements:(i+1)*num_elements+num_elements]
        D += distance(pos1,pos2)
    #print(D)

    path_ratio = D/best_path
    paths_ratios.append(path_ratio)
    #print("Path ratio ",path_ratio)
# function for counting diferences between elemnet - 3, 2

average_path = statistics.mean(paths_ratios)
print("Average path ratio ", average_path)



# Write the array to the CSV file
with open(result_file_path, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerows(result)

print(f"Data has been written to {result_file_path}")

# for i,r_input in enumerate(input_data):
#     r_output = output_data[i]
#     if i==1:
#         print(i,r_input,r_output,r_input[3:5],int(r_output[1]))
#
# print(len(input_data))

# 3D plot
