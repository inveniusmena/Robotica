import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import median_filter
from datetime import datetime

# Function to remove outliers using IQR
def remove_outliers_iqr(data):
    q1 = np.percentile(data, 25)  # First quartile (25th percentile)
    q3 = np.percentile(data, 75)  # Third quartile (75th percentile)
    iqr = q3 - q1
    lower_bound = q1 - 1.5 * iqr
    upper_bound = q3 + 1.5 * iqr
    # Mask where values are within bounds
    valid_mask = (data >= lower_bound) & (data <= upper_bound)
    return valid_mask  # Return a boolean mask for valid values

# Load the data from the .txt file
file_path = 'LAB1_7.TXT'
data = np.loadtxt(file_path)

# Extract columns
time_stamps = data[:, 0] / 1000000  # Convert microseconds to seconds
acc_x = data[:, 1]
gyro_pitch = data[:, 2]
gyro_yaw = data[:, 3]

# Define the intervals for outlier removal (adjust as needed)
t_min_x, t_max_x = 5, 7.5  # Interval for roll: 1.25 to 3 seconds
interval_mask_x = (time_stamps >= t_min_x) & (time_stamps <= t_max_x)

t_min_y, t_max_y = 5, 6  # Interval for y: 4.5 to 6 seconds
interval_mask_y = (time_stamps >= t_min_y) & (time_stamps <= t_max_y)

t_min_z, t_max_z = 5, 6  # Interval for z: 1.25 to 3 seconds
interval_mask_z = (time_stamps >= t_min_z) & (time_stamps <= t_max_z)

# Apply separate masking and outlier removal for each dataset
# x data outlier removal and masking
x_interval = acc_x[interval_mask_x]
outlier_mask_x = remove_outliers_iqr(x_interval)
valid_time_stamps_x = time_stamps[interval_mask_x][outlier_mask_x]
valid_acc_x = acc_x[interval_mask_x][outlier_mask_x]

# Yaw data outlier removal and masking
yaw_interval = gyro_yaw[interval_mask_y]
outlier_mask_yaw = remove_outliers_iqr(yaw_interval)
valid_time_stamps_yaw = time_stamps[interval_mask_y][outlier_mask_yaw]
valid_gyro_yaw = gyro_yaw[interval_mask_y][outlier_mask_yaw]

# Pitch data outlier removal and masking
pitch_interval = gyro_pitch[interval_mask_z]
outlier_mask_pitch = remove_outliers_iqr(pitch_interval)
valid_time_stamps_pitch = time_stamps[interval_mask_z][outlier_mask_pitch]
valid_gyro_pitch = gyro_pitch[interval_mask_z][outlier_mask_pitch]

# For the entire dataset, create a mask to keep the data that is not removed
# Apply the mask for each dataset independently
mask_roll = np.ones_like(time_stamps, dtype=bool)
mask_roll[interval_mask_x] = outlier_mask_x  # for roll data

mask_yaw = np.ones_like(time_stamps, dtype=bool)
mask_yaw[interval_mask_y] = outlier_mask_yaw  # for yaw data

mask_pitch = np.ones_like(time_stamps, dtype=bool)
mask_pitch[interval_mask_z] = outlier_mask_pitch  # for pitch data

# Apply each mask separately to filter out outliers
filtered_time_stamps_roll = time_stamps[mask_roll]
filtered_acc_x = acc_x[mask_roll]

filtered_time_stamps_yaw = time_stamps[mask_yaw]
filtered_gyro_yaw = gyro_yaw[mask_yaw]

filtered_time_stamps_pitch = time_stamps[mask_pitch]
filtered_gyro_pitch = gyro_pitch[mask_pitch]

# Apply a median filter to the roll, yaw, and pitch data (after outliers removed)
window_size = 5  # Define the window size for the median filter
filtered_acc_x_smooth = median_filter(filtered_acc_x, size=window_size)
filtered_gyro_yaw_smooth = median_filter(filtered_gyro_yaw, size=window_size)
filtered_gyro_pitch_smooth = median_filter(filtered_gyro_pitch, size=window_size)

# Plot the results
plt.figure(figsize=(12, 8))

# Roll
plt.subplot(3, 1, 1)
plt.plot(time_stamps, acc_x, label="Original x Data", alpha=0.7)
plt.plot(filtered_time_stamps_roll, filtered_acc_x_smooth, label="Filtered x Data (Outliers Removed and Smoothed)", linewidth=2, alpha=0.9)
plt.axvspan(t_min_x, t_max_x, color='yellow', alpha=0.2, label="Filtered Interval")
plt.title("X Data Filtering and Median Smoothing (Outliers Removed)")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (mG)")
plt.legend()
plt.grid()

# Yaw
plt.subplot(3, 1, 2)
plt.plot(time_stamps, gyro_yaw, label="Original y Data", alpha=0.7)
plt.plot(filtered_time_stamps_yaw, filtered_gyro_yaw_smooth, label="Filtered y Data (Outliers Removed and Smoothed)", linewidth=2, alpha=0.9)
plt.axvspan(t_min_y, t_max_y, color='yellow', alpha=0.2, label="Filtered Interval")
plt.title("Y Data Filtering and Median Smoothing (Outliers Removed)")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (mG)")
plt.legend()
plt.grid()

# Pitch
plt.subplot(3, 1, 3)
plt.plot(time_stamps, gyro_pitch, label="Original z Data", alpha=0.7)
plt.plot(filtered_time_stamps_pitch, filtered_gyro_pitch_smooth, label="Filtered z Data (Outliers Removed and Smoothed)", linewidth=2, alpha=0.9)
plt.axvspan(t_min_z, t_max_z, color='yellow', alpha=0.2, label="Filtered Interval")
plt.title("Z Data Filtering and Median Smoothing (Outliers Removed)")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (mG)")
plt.legend()
plt.grid()

plt.tight_layout()
current_timestamp = datetime.now()
plt.savefig(f'aceleracao {current_timestamp}.png')

# Transpose the data
data = zip(filtered_acc_x_smooth, filtered_gyro_yaw_smooth, filtered_gyro_pitch_smooth)

with open("filtered_data_outliers.txt", "w") as file:
    for row in data:
        # Join each item in the row with a tab character and write to the file
        file.write("\t".join(map(str, row)) + "\n")
