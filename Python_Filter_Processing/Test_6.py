import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

"""
-This test is to show the effects of using a centered moving average filter with dynamic window size adjustment and removing an offset
-This is the same concept as Test_4.py and Test_5.py; as such I also recommend this
-Window size is 100 but can be adjusted to however big is needed. 
-The bigger the window size, the greater the smoothing
-Can be improved upon to accept real-time data points


"""

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
offset = 1 # adjust offset here as needed
time = df['adjusted_timestamp']
acc_z = df['accZ'] - offset

filtered_acc_z = centered_moving_average(df['accZ'], window_size, offset)

plt.figure(figsize=(20, 15))
plt.plot(time, acc_z, color='gray', alpha=0.5, label='Raw Data')
plt.plot(time, filtered_acc_z, color='red', linewidth=2, label=f'Smoothed ({window_size}-point MA)')

# Customize plot
plt.title("Accelerometer Data: Raw vs. Smoothed", fontsize=14)
plt.xlabel("Time (s)", fontsize=12)  # Label matches your timestamp units
plt.ylabel("Acceleration (m/s²)", fontsize=12)
plt.legend()
plt.grid(alpha=0.3)
plt.tight_layout()  # Prevents label cutoff
plt.show()