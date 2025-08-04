import numpy as np

class GPSFilter:
    def __init__(self, window_size=5): # Assign default values; can be overriden
        self.gps_readings = []
        self.satellite_count = []
        self.weights = np.empty(window_size, dtype=float)

    def _compute_weights(self): # Calculate weights for softmax filter
        np_satellite_count = np.array(self.satellite_count) # Typecast self.satellite_count as a numpy array
        exp_weights = np.exp(np_satellite_count - np.max(np_satellite_count)) # Subtract max 
        return exp_weights / np.sum(exp_weights)  # Return normalised weights

    def update(self, new_gps, new_sat_count):
        self.gps_readings.append(float(new_gps))
        self.satellite_count.append(int(new_sat_count))

        if len(self.gps_readings) > len(self.weights) and len(self.satellite_count) > len(self.weights):
            self.gps_readings.pop(0)  # Keep gps_readings size = window_size; remove oldest sample
            self.satellite_count.pop(0) 
        
        if len(self.gps_readings) < len(self.weights):
            return new_gps # Not enough data points; Return raw unfiltered value
        else:
            self.weights[:] = self._compute_weights()
            return float(np.round(np.dot(np.array(self.gps_readings), self.weights), 4))