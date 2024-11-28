import numpy as np
from scipy.ndimage import median_filter
from scipy.ndimage import gaussian_filter
import matplotlib.pyplot as plt
from datetime import datetime


file_path = '/workspaces/Robotica/LAB1_7.TXT'

# Load the data from the .txt file
data = np.loadtxt(file_path)

time_stamps = data[:,0]

# Apply the median filter to each of the last 6 columns
filtered_data_3 = data.copy()
filtered_data_5 = data.copy()
filtered_data_7 = data.copy()
for i in range(1,7):  
    filtered_data_3[:, i] = median_filter(data[:, i], size=3)
    filtered_data_5[:, i] = median_filter(data[:, i], size=5)
    filtered_data_7[:, i] = median_filter(data[:, i], size=7)

gaussian_filtered_data = data.copy()
for i in range(1,7):  
    gaussian_filtered_data[:, i] = gaussian_filter(data[:,i], sigma=2)

#Export gaussian filter data

# Open a file in write mode
with open("filtered_data.txt", "w") as file:
    for row in gaussian_filtered_data:
        # Join each item in the row with a tab character and write to the file
        file.write("\t".join(map(str, row)) + "\n")

# Plot the results
y_axis_legend=["Acceleration in x", "Acceleration in y", "Acceleration in z", "Angular Velocity in Roll", "Angular Velocity in Pitch", "Angular Velocity in Yaw"]
y_axis_units = ["(mG)","(°/s)"]
for i in range(1,7):
    plt.figure(figsize=(10, 6)) #new figure each plot
    plt.plot(time_stamps, data[:, i],label='Original Data',  )
    plt.plot(time_stamps, filtered_data_3[:, i], label='Median Filter (window size 3)')
    #plt.plot(time_stamps, filtered_data_5[:, i], label='Filtered Data (window size 5)')
    #plt.plot(time_stamps, filtered_data_7[:, i], label='Filtered Data (window size 7)')
    plt.plot(time_stamps,gaussian_filtered_data[:, i], label='Gaussian Filter')
    plt.legend()
    plt.xlabel("Time (ms)")
    plt.ylabel(f"{y_axis_legend[i-1]} {y_axis_units[int((i-1)/3)]} ")
    plt.title(f'Different Filters for {y_axis_legend[i-1]}')

    timestamp = datetime.now()
    # Save the plot as an image file
    plt.savefig(f'graficos/task2_{y_axis_legend[i-1]} {timestamp}.png')
    plt.show()


