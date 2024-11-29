import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import median_filter

# Function to remove outliers using IQR and replace them with the mean of the interval
def replace_outliers_with_mean(data):
    q1 = np.percentile(data, 25)  # First quartile (25th percentile)
    q3 = np.percentile(data, 75)  # Third quartile (75th percentile)
    iqr = q3 - q1
    lower_bound = q1 - 1.5 * iqr
    upper_bound = q3 + 1.5 * iqr
    
    # Identify outliers
    outliers_mask = (data < lower_bound) | (data > upper_bound)
    
    # Replace outliers with the mean of the interval
    data[outliers_mask] = np.mean(data)
    
    return data, outliers_mask  # Return the modified data with outliers replaced

# Load the data from the .txt file
file_path = 'LAB1_7.TXT'
data = np.loadtxt(file_path)

# Extract columns
time_stamps = data[:, 0] / 1000000  # Convert microseconds to seconds
gyro_roll = data[:, 1]
gyro_pitch = data[:, 2]
gyro_yaw = data[:, 3]

# Define the intervals for outlier removal (adjust as needed)
t_min_roll, t_max_roll = 5, 7.5    # Interval for roll: 1.25 to 3 seconds
interval_mask_roll = (time_stamps >= t_min_roll) & (time_stamps <= t_max_roll)
roll_interval = gyro_roll[interval_mask_roll]

t_min_yaw, t_max_yaw = 5, 6  # Interval for yaw: 4.5 to 6 seconds
interval_mask_yaw = (time_stamps >= t_min_yaw) & (time_stamps <= t_max_yaw)
yaw_interval = gyro_yaw[interval_mask_yaw]

t_min_pitch, t_max_pitch = 5, 6  # Interval for pitch: 1.25 to 3 seconds
interval_mask_pitch = (time_stamps >= t_min_pitch) & (time_stamps <= t_max_pitch)
pitch_interval = gyro_pitch[interval_mask_pitch]

# Apply outlier replacement for roll, yaw, and pitch
gyro_roll_no_outliers = np.copy(gyro_roll)
gyro_yaw_no_outliers = np.copy(gyro_yaw)
gyro_pitch_no_outliers = np.copy(gyro_pitch)

# Apply outlier replacement for roll, yaw, and pitch
gyro_roll_no_outliers[interval_mask_roll], roll_outlier_mask = replace_outliers_with_mean(roll_interval)
gyro_yaw_no_outliers[interval_mask_yaw], yaw_outlier_mask = replace_outliers_with_mean(yaw_interval)
gyro_pitch_no_outliers[interval_mask_pitch], pitch_outlier_mask = replace_outliers_with_mean(pitch_interval)

# Apply a median filter to the roll, yaw, and pitch data (after outliers replaced)
window_size = 7  # Define the window size for the median filter
gyro_roll_smooth = median_filter(gyro_roll_no_outliers, size=window_size)
gyro_yaw_smooth = median_filter(gyro_yaw_no_outliers, size=window_size)
gyro_pitch_smooth = median_filter(gyro_pitch_no_outliers, size=window_size)

# Plot the results
plt.figure(figsize=(12, 8))

# Roll
plt.subplot(3, 1, 1)
plt.plot(time_stamps, gyro_roll, label="Original Roll Data", alpha=0.7)
plt.plot(time_stamps, gyro_roll_smooth, label="Filtered Roll Data (Outliers Replaced and Smoothed)", linewidth=2, alpha=0.9)
plt.axvspan(t_min_roll, t_max_roll, color='yellow', alpha=0.2, label="Filtered Interval")
plt.scatter(time_stamps[interval_mask_roll][roll_outlier_mask], gyro_roll[interval_mask_roll][roll_outlier_mask], color='red', label="Roll Outliers", zorder=5)
plt.title("Roll Data Filtering and Median Smoothing (Outliers Replaced)")
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (degrees/s)")
plt.legend()
plt.grid()

# Yaw
plt.subplot(3, 1, 2)
plt.plot(time_stamps, gyro_yaw, label="Original Yaw Data", alpha=0.7)
plt.plot(time_stamps, gyro_yaw_smooth, label="Filtered Yaw Data (Outliers Replaced and Smoothed)", linewidth=2, alpha=0.9)
plt.axvspan(t_min_yaw, t_max_yaw, color='yellow', alpha=0.2, label="Filtered Interval")
plt.scatter(time_stamps[interval_mask_yaw][yaw_outlier_mask], gyro_yaw[interval_mask_yaw][yaw_outlier_mask], color='red', label="Yaw Outliers", zorder=5)
plt.title("Yaw Data Filtering and Median Smoothing (Outliers Replaced)")
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (degrees/s)")
plt.legend()
plt.grid()

# Pitch
plt.subplot(3, 1, 3)
plt.plot(time_stamps, gyro_pitch, label="Original Pitch Data", alpha=0.7)
plt.plot(time_stamps, gyro_pitch_smooth, label="Filtered Pitch Data (Outliers Replaced and Smoothed)", linewidth=2, alpha=0.9)
plt.axvspan(t_min_pitch, t_max_pitch, color='yellow', alpha=0.2, label="Filtered Interval")
plt.scatter(time_stamps[interval_mask_pitch][pitch_outlier_mask], gyro_pitch[interval_mask_pitch][pitch_outlier_mask], color='red', label="Pitch Outliers", zorder=5)
plt.title("Pitch Data Filtering and Median Smoothing (Outliers Replaced)")
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (degrees/s)")
plt.legend()
plt.grid()

plt.tight_layout()

# Transpose the data
data = zip(gyro_roll_smooth, gyro_pitch_smooth, gyro_yaw_smooth)

with open("filtered_data_outliers.txt", "w") as file:
    for row in data:
        # Join each item in the row with a tab character and write to the file
        file.write("\t".join(map(str, row)) + "\n")

plt.show()