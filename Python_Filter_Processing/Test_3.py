import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

"""
This test is to show the effects of using a convolution filter with an equal weighted kernel with a window size of 9.
4 intial and 4 final values are not included due to the effects of the convolution filter

"""

df = pd.read_excel('Datalogger.xlsx', sheet_name='Sheet2') # Import entire Excel file as a dataframe
time = df['adjusted_timestamp']
acc_z = df['accZ']

trimmed_time = time[4:-4]  # Length = len(time) - 8
trimmed_acc_z = acc_z[4:-4]  # Trim raw data to match filtered_x

nine_kernel = np.ones(9)/9
filtered_acc_z = np.convolve(acc_z, nine_kernel, mode='valid')

plt.figure(figsize=(12, 6))
plt.plot(trimmed_time, trimmed_acc_z, color='gray', alpha=0.5, label='Raw Data')
plt.plot(trimmed_time, filtered_acc_z, color='red', linewidth=2, label='Smoothed (9-point MA Numpy.convolve)')

# Customize plot
plt.title("Accelerometer Data: Raw vs. Smoothed", fontsize=14)
plt.xlabel("Time", fontsize=12)  # Label matches your timestamp units
plt.ylabel("Acceleration (m/s²)", fontsize=12)
plt.legend()
plt.grid(alpha=0.3)
plt.tight_layout()  # Prevents label cutoff
plt.show()
