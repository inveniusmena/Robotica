import numpy as np
from scipy.ndimage import median_filter
import matplotlib.pyplot as plt


file_path = '/workspaces/Robotica/LAB1_7.TXT'

# Load the data from the .txt file
data = np.loadtxt(file_path)

time = data[:,0]

# Apply the median filter to each of the last 6 columns
filtered_data_3 = data.copy()
filtered_data_5 = data.copy()
filtered_data_7 = data.copy()
for i in range(1,7):  
    print(i)
    filtered_data_3[:, i] = median_filter(data[:, i], size=3)
    filtered_data_5[:, i] = median_filter(data[:, i], size=5)
    filtered_data_7[:, i] = median_filter(data[:, i], size=7)


# Plot the results
y_axis_legend=["accelaration in x", "acceleration in y", "acceleration in z", "angular velocity in roll", "angular velocity in pitch", "angular velocity in yaw"]

for i in range(2,7):
    plt.figure(figsize=(10, 6)) #new figure each plot
    plt.plot(time, data[:, i],label='Original Data',  )
    plt.plot(time, filtered_data_3[:, i], label='Filtered Data (window size 3)')
    plt.plot(time, filtered_data_5[:, i], label='Filtered Data (window size 5)')
    plt.plot(time, filtered_data_7[:, i], label='Filtered Data (window size 7)')
    plt.legend()
    plt.xlabel("Time (ms)")
    plt.ylabel(f"{y_axis_legend[i-1]} (m/ms)")
    plt.title(f'Median Filter for {y_axis_legend[i-1]} with Different Window Sizes')

    # Save the plot as an image file
    plt.savefig(f'task2_column{i}.png')
    plt.show()


