import pandas as pd
import matplotlib.pyplot as plt
import time
from RealTimeGaussianFilter import RealTimeGaussianFilter as gaussian_filter

"""
Simulates the functionality of the gaussian filter. Values from dataframe are fed to the filter with 
an artificially induced delay of 200ms to simulate sensor update frequency.
"""

window_size = 5 # Adjust size of filter window
sigma = 1 # Adjust the 'width' of bell curve / how much neighbouring values matter
filter = gaussian_filter(window_size, sigma) # Create an instance of the gaussian_filter

df = pd.read_csv('T0122064.csv')
column_names = ['accX', 'accY', 'accZ', 'gyroX', 'gyroY', 'gyroZ', 'magX', 'magY', 'magZ']
name = int(input("Enter the column to apply filter 0 - 8 : "))
raw_data = df[column_names[name]]

filtered_data = [] # Store individual data points; not necessary for filter, only used for this program
count = 0

for sample in raw_data.head(1000):
    filtered_value = filter.update(sample) # Input sensor data into filter; method returns a float of 4 decimal places
    filtered_data.append(filtered_value) 
    count += 1

    print(f"{count} New sample: {sample:.6f} | Filtered: {filtered_value:.6f} | Buffer: {list(filter.buffer)}")
    
    time.sleep(0.2) # Simulate live sensor data feed

filtered_dataDF = pd.DataFrame(filtered_data, columns=['filtered'])
print()
print(raw_data.head(1000))
print()
print(filtered_dataDF)  # First 4 values are raw (buffer filling only takes 1 second), then smoothed
print()

plt.figure(figsize=(20, 15))
plt.plot(df['adjusted'].head(1000), raw_data, color='gray', alpha=0.5, label='Raw Data')
plt.plot(df['adjusted'].head(1000), filtered_dataDF, color='red', linewidth=1.5, label=f'Smoothed')

# Customize plot
plt.title("Data: Raw vs. Smoothed", fontsize=14)
plt.xlabel("Time (s)", fontsize=12) 
plt.ylabel("", fontsize=12) # Name of Y-axis has been left blank intentionally
plt.legend()
plt.grid(alpha=0.3)
plt.tight_layout()  # Prevents label cutoff
plt.show()