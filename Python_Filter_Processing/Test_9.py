import numpy as np

class RealTimeGaussianFilter:
    def __init__(self, window_size=5, sigma=1.0):
        self.buffer = [] 
        self.weights =  self._compute_weights(window_size, sigma)

    def _compute_weights(self, window_size, sigma):
        x = np.arange(-(window_size // 2), window_size // 2 + 1)
        weights = np.exp(-(x ** 2) / (2 * sigma ** 2))
        return weights / weights.sum() # Return normalised weights
    
    def update(self, new_sample):
        self.buffer.append(new_sample)
        if len(self.buffer) > len(self.weights):
            self.buffer.pop(0)  # Keep buffer size = window_size; remove oldest sample
        
        if len(self.buffer) < len(self.weights):
            return new_sample  # Not enough data yet
        else:
            return float(np.round(np.dot(self.buffer, self.weights), 3))

window_size = 5
sigma = 1 # Control how wide bell curve is / how much neighbouring values matter

gaussian_filter = RealTimeGaussianFilter(window_size, sigma)

data_stream = [1.2, 1.5, 1.3, 1.7, 1.4, 1.6, 1.1]  # Simulated real-time data

smoothed = []
for sample in data_stream:
    smoothed.append(gaussian_filter.update(sample))


print(smoothed)  # First 2-3 values are raw (buffer filling), then smoothed.