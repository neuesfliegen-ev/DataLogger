import numpy as np

class RealTimeGaussianFilter:
    def __init__(self, window_size=5, sigma=1.0): # Assign default values; can be overriden
        self.buffer = [] 
        self.weights =  self._compute_weights(window_size, sigma)

    def _compute_weights(self, window_size, sigma): # Calculate bell curve
        x = np.arange(-(window_size // 2), window_size // 2 + 1)
        weights = np.exp(-(x ** 2) / (2 * sigma ** 2))
        return weights / weights.sum() # Return normalised weights
    
    def update(self, new_sample): 
        self.buffer.append(float(new_sample))

        if len(self.buffer) > len(self.weights):
            self.buffer.pop(0)  # Keep buffer size = window_size; remove oldest sample
        
        if len(self.buffer) < len(self.weights):
            return new_sample  # Not enough data yet, return same exact sample
        else:
            return float(np.round(np.dot(np.array(self.buffer), self.weights), 4))