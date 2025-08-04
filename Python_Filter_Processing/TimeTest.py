import pandas as pd
import time
from datetime import datetime

from RealTimeGaussianFilter import RealTimeGaussianFilter as gaussian_filter
from GPSSoftmaxFilter import GPSFilter as gps_filter

"""
Records the time taken for both filters to act on all 12 sensor values per update. The actual time taken
will likely be a few microseconds less.

Program also outputs a .csv file of the recorded elapsed time. 
"""

# Initialize filters
window_size = 5
sigma = 1 # Control how wide bell curve is / how much neighbouring values matter
gaussianFilter = gaussian_filter(window_size, sigma)
gpsFilter = gps_filter(window_size)

# Load csv
df = pd.read_csv('T0122064.csv')

# Timing related variables
timings = []  # Store individual timings
count = 0

for i in range(len(df)):
    count += 1
    start_time = time.perf_counter()

    # Gaussian filtering
    for j in range(2, 11):

        filtered_value = gaussianFilter.update(df.iloc[i, j])
    
    # GPS softmax filtering
    for j in range(11, 14):

        filtered_value = gpsFilter.update(df.iloc[i,j], df.iloc[i, 15])
        
    elapsed = time.perf_counter() - start_time  # Calculate duration
    timings.append(elapsed)
    print(f"{count} Time elapsed: {elapsed:.6f} sec")

    time.sleep(0.2)

# Summary statistics
avg_time = sum(timings) / len(timings)
print()
print(f"Average time per sample: {avg_time:.6f} sec")
print(f"Max time: {max(timings):.6f} sec | Min time: {min(timings):.6f} sec")
print()

# === Save to CSV ===

# Generate unique filename using timestamp
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename = f"timings_{timestamp}.csv"

# Save timings to CSV
timing_df = pd.DataFrame({
    "Sample": range(1, len(timings) + 1),
    "ElapsedTime_sec": timings
})
timing_df.to_csv(csv_filename, index=False)

print(f"\nTiming data saved to '{csv_filename}'")