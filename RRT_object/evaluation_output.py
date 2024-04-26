# Michal Biro
# 14.4 2024
#
# script for evaluating outputs from experiment - reading files 2DOF and 3DOF_output_data
import csv

# Define the file name
result_file_path = "data_files/result_XYA.csv"
data_file_path = "data_files/XYA_output_data.csv" # Define the file name


output_data = []
with open(data_file_path, 'r', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        # Convert each element in the row from string to integer
        row = [float(x) for x in row]
        # Append the row to the data list
        output_data.append(row)

# Load input data
file_path = "data_files/input.csv" # Define the file name
input_data = []
with open(file_path, 'r', newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        # Convert each element in the row from string to integer
        row = [int(x) for x in row]
        # Append the row to the data list
        input_data.append(row)

object_size_data = []

for row in input_data:
    # Extract the values at the 3rd and 4th positions
    w = row[3]  # 3rd position (index 2)
    h = row[4]  # 4th position (index 3)

    # Append the values to the list
    object_size_data.append((w, h))

object_size_list = sorted(list(set(object_size_data)))

object_size_data = []
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
    object_size_data.append((w, h,sol))
    # if time < 5: global_time += time
    # else: global_time += 5

result=[]
global_success = 0
all_cases = 0
for w,h in object_size_list:
    success = object_size_data.count((w,h,1))
    failure = object_size_data.count((w, h, 0))
    num_cases = success + failure
    if num_cases == 0:
        success_rate = 0
    else:
        success_rate = round((success/num_cases)*100,2)
    #print(w,h,success,failure,num_cases,success_rate)
    result.append([w,h,success,failure,num_cases,success_rate])
    global_success += success
    all_cases += num_cases

print("Number of successful cases: ",global_success,"/",all_cases)
print("Success rate in all cases: ",(global_success/all_cases)*100)
print("Average time for finding path: ", round(global_time/all_cases,2))


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
