import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

"""
-This test is to show the effects of using the rolling mean function from pandas; it has built in edge handling.
-Window size is 5 but can be adjusted to however big is needed. 
-The bigger the window size, the greater the smoothing

I personally recommend this since there is no artifacting happening at the edges due to the window size
being dynamically adjusted at the edges.

"""

df = pd.read_excel('Datalogger.xlsx', sheet_name='Sheet2') # Import entire Excel file as a dataframe
time = df['adjusted_timestamp']
acc_z = df['accZ']

df['filtered_acc_z'] = df['accZ'].rolling(100, center=True, min_periods=1).mean() # "min_periods=1" returns partial values (e.g., 3-point avg for the 2nd value).

# print to check the length of each column
# not neccessary for the functionality of the code
print(f"Time: {len(df['adjusted_timestamp'])}, acc_z: {len(df['accZ'])}, filtered: {len(df['filtered_acc_z'])}") 

plt.figure(figsize=(12, 6))
plt.plot(time, acc_z, color='gray', alpha=0.5, label='Raw Data')
plt.plot(time, df['filtered_acc_z'], color='red', linewidth=2, label='Smoothed (100-point MA)')

# Customize plot
plt.title("Accelerometer Data: Raw vs. Smoothed", fontsize=14)
plt.xlabel("Time", fontsize=12)  # Label matches your timestamp units
plt.ylabel("Acceleration (m/s²)", fontsize=12)
plt.legend()
plt.grid(alpha=0.3)
plt.tight_layout()  # Prevents label cutoff
plt.show()