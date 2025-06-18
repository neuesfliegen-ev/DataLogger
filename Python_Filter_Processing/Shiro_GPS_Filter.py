import numpy as np

def weighted_gps_filter(gps_readings, satellite_counts):
    """
    Applies a weighted average filter on GPS readings using normalized satellite counts as weights.

    Parameters:
    - gps_readings: list or array of GPS readings (e.g., latitude or longitude)
    - satellite_counts: list or array of satellite counts for each reading

    Returns:
    - filtered_reading: weighted average of GPS readings
    """
    gps_readings = np.array(gps_readings)
    satellite_counts = np.array(satellite_counts)

    # Use softmax for smoother weighting (optional, can replace with raw normalization)
    exp_weights = np.exp(satellite_counts)
    weights = exp_weights / np.sum(exp_weights)  

    # Apply weighted average
    filtered_reading = np.sum(weights * gps_readings)
    return filtered_reading

# Example usage:
gps_data = [29.9871, 29.9875, 29.9873, 29.9878, 29.9874]  # Sample latitudes n-2 n -1 n n +1 n +2     1 2 3 4 5 6 7 8 9
sat_counts = [5, 7, 10, 12, 8]  # Number of satellites per reading

filtered_lat = weighted_gps_filter(gps_data, sat_counts)
print(f"Filtered Latitude: {filtered_lat:.7f}")