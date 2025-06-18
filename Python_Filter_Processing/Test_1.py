import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

"""
This test is to show the effects of using a convolution filter with a center-weighted symmetric kernel.

!! Not recommeneded due to presence of major artifacting at the edges !!

"""

df = pd.read_excel('Datalogger.xlsx', sheet_name='Sheet2') # Import entire Excel file as a dataframe
time = df['adjusted_timestamp']
acc_z = df['accZ']

five_kernel = np.array([0.1, 0.2, 0.4, 0.2, 0.1]) # Center-weighted symmetric kernel

filtered_acc_z = np.convolve(acc_z, five_kernel, mode='same') # "mode='same' pads the dataframe with zeros so that input length = output length"

plt.figure(figsize=(12, 6))
plt.plot(time, acc_z, color='gray', alpha=0.5, label='Raw Data')
plt.plot(time, filtered_acc_z, color='red', linewidth=2, label='Smoothed (5-point MA Center-weighted symmetric kernel)')

# Customize plot
plt.title("Accelerometer Data: Raw vs. Smoothed", fontsize=14)
plt.xlabel("Time", fontsize=12)  # Label matches your timestamp units
plt.ylabel("Acceleration (m/s²)", fontsize=12)
plt.legend()
plt.grid(alpha=0.3)
plt.tight_layout()  # Prevents label cutoff
plt.show()
