import pandas as pd
import matplotlib.pyplot as plt
import time
from GPSSoftmaxFilter import GPSFilter as gps_filter

"""
Simulates the functionality of the softmax filter. Values from dataframe are fed to the filter with 
an artificially induced delay of 200ms to simulate sensor update frequency.
"""

window_size = 5 # Adjust size of filter window
filter = gps_filter(window_size) # Create an instance of the gps_filter

df = pd.read_csv('T0122064.csv')
column_names = ['latitude', 'longitude', 'gpsAltitude']
name = int(input("Enter the column to apply filter 0 - 2 : "))
raw_data = df[column_names[name]]
gps_count = df['SatCount']

filtered_data = [] # Store individual data points; not necessary for filter, only used for this program
count = 0

for i in range(len(df)):
    filtered_value = filter.update(raw_data.iloc[i], gps_count.iloc[i]) # Input sensor data into filter; method returns a float of 4 decimal places
    filtered_data.append(filtered_value) 
    count += 1

    print(f"{count} New sample: {raw_data.iloc[i]:.6f} | Filtered: {filtered_value:.6f} | GPS Readings: {list(filter.gps_readings)} | Satellite Count: {list(filter.satellite_count)}")
    
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