import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_excel('Datalogger.xlsx', sheet_name='Sheet2') # Import entire Excel file as a dataframe

time = df['adjusted_timestamp']
acc_x = df['accX']
acc_y = df['accY']
acc_z = df['accZ']

trimmed_time = time[4:-4]  # Length = len(time) - 4
trimmed_acc_z = acc_z[4:-4]  # Trim raw data to match filtered_x

five_kernel = np.ones(5)/5
nine_kernel = np.ones(9)/9
filtered_y = np.convolve(acc_z, nine_kernel, mode='valid')

print(f"Time: {len(trimmed_time)}, acc_z: {len(trimmed_acc_z)}, filtered: {len(filtered_y)}, kernel: {nine_kernel}")

plt.figure(figsize=(12, 6))
plt.plot(trimmed_time, trimmed_acc_z, color='gray', alpha=0.5, label='Raw Data')
plt.plot(trimmed_time, filtered_y, color='red', linewidth=2, label='Smoothed (5-point MA)')

# Customize plot
plt.title("Accelerometer Data: Raw vs. Smoothed", fontsize=14)
plt.xlabel("Time", fontsize=12)  # Label matches your timestamp units
plt.ylabel("Acceleration (m/s²)", fontsize=12)
plt.legend()
plt.grid(alpha=0.3)
plt.tight_layout()  # Prevents label cutoff
plt.show()
