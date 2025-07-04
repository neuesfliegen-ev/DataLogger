import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

"""
This test is to show the effects of using a convolution filter with an equal weighted kernel.
Zeros are added at the edges.

!! Not recommeneded due to presence of major artifacting at the edges !!

"""

df = pd.read_excel('Datalogger.xlsx', sheet_name='Sheet2') # Import entire Excel file as a dataframe
time = df['adjusted_timestamp']
acc_z = df['accZ']
window_size = 5 # Adjust window size here

kernel = np.ones(window_size)/window_size # Create an array of len(window_size) whose sum equals 1
padded_data = np.pad(acc_z, (window_size//2, window_size//2), mode='constant')  # Add window_size // 2 number of zeros on each side of the data
filtered_acc_z = np.convolve(padded_data, kernel, mode='valid')  # No trimming needed; filtering removes the first two and last two values

plt.figure(figsize=(12, 6))
plt.plot(time, acc_z, color='gray', alpha=0.5, label='Raw Data')
plt.plot(time, filtered_acc_z, color='red', linewidth=2, label=f'Smoothed ({window_size}-point MA Equal Weighted Kernel)')

# Customize plot
plt.title("Accelerometer Data: Raw vs. Smoothed", fontsize=14)
plt.xlabel("Time", fontsize=12)  # Label matches your timestamp units
plt.ylabel("Acceleration (m/s²)", fontsize=12)
plt.legend()
plt.grid(alpha=0.3)
plt.tight_layout()  # Prevents label cutoff
plt.show()

