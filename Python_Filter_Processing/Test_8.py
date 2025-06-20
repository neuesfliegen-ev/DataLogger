import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

"""
-This test is to compare the effects of using a causal moving average against that of a centered moving average
-Window size is 100 but can be adjusted to however big is needed. 
-Can be improved upon to accept real-time data points.


"""

def causal_moving_average(data, window, offset):
    smoothed = np.zeros_like(data)

    for i in range(len(data)):
        start = max(0, i - window)
        smoothed[i] = np.mean(data[start : i + 1] - offset)

    return smoothed 

def centered_moving_average(data, window_size, offset):
    half_win = window_size // 2  # e.g., 2 for a 5-point window
    smoothed = np.zeros_like(data) # create a new array of zeros with the same size as input 'data' array
    
    for i in range(len(data)):
        # Determine the dynamic window bounds
        start = max(0, i - half_win)          # Avoid negative indices
        end = min(len(data), i + half_win + 1) # Avoid exceeding array length
        smoothed[i] = np.mean(data[start:end] - offset)
    
    return smoothed

df = pd.read_excel('Datalogger.xlsx', sheet_name='Sheet2') # Import entire Excel file as a dataframe
window_size = 100 # adjust window size here as needed
offset = 1.005 # adjust offset here as needed
plot_start = 0 # adjust start of plot
plot_end = 27000 # adjust end of plot
time = df['adjusted_timestamp']
acc_z = df['accZ'] - offset
expected_acc_z = 0 # set expected value of z

filtered_causal_z = causal_moving_average(df['accZ'], window_size, offset)
filtered_centered_z = centered_moving_average(df['accZ'], window_size, offset)
mean_filtered_causal_z = np.mean(causal_moving_average(df['accZ'], window_size, offset))
mean_filtered_centered_z = np.mean(centered_moving_average(df['accZ'], window_size, offset))

print("\nMean of Filtered Causal Z = ", mean_filtered_causal_z, "\nMean of Filtered Centered Z = ", mean_filtered_centered_z )
print("\nDelta Causal = ", abs(expected_acc_z - mean_filtered_causal_z), "\nDelta Centered = ", abs(expected_acc_z - mean_filtered_centered_z))
# delta is just a quantitative measurement of how well the filter gets to the expected value
print("\n")

plt.figure(figsize=(20, 15))
plt.plot(time[plot_start:plot_end], acc_z[plot_start:plot_end], color='gray', alpha=0.5, label='Raw Data')
plt.plot(time[plot_start:plot_end], filtered_causal_z[plot_start:plot_end], color='red', linewidth=2, label=f'Smoothed ({window_size}-point Causal MA)')
plt.plot(time[plot_start:plot_end], filtered_centered_z[plot_start:plot_end], color='blue', linewidth=2, label=f'Smoothed ({window_size}-point Centered MA)')

# Customize plot
plt.title("Accelerometer Data: Raw vs. Smoothed", fontsize=14)
plt.xlabel("Time (s)", fontsize=12)  # Label matches your timestamp units
plt.ylabel("Acceleration (m/s²)", fontsize=12)
plt.legend()
plt.grid(alpha=0.3)
plt.tight_layout()  # Prevents label cutoff
plt.show()
