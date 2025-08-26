#!/usr/bin/env python3
"""
Animated 3D Flight Viewer - Shows flight path gradually over time.
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button, TextBox
import numpy as np
from pathlib import Path
from data_loader import FlightDataLoader


def load_npu_data_fixed(csv_path):
    """Load NPU data with format fixes for corrupted roll/pitch/yaw columns."""
    import pandas as pd
    import re
    
    # Read raw lines to fix format issues
    with open(csv_path, 'r') as f:
        lines = f.readlines()
    
    header = lines[0].strip()
    fixed_lines = [header]
    
    print(f"Fixing NPU data format issues...")
    fixed_count = 0
    
    for i, line in enumerate(lines[1:], 1):
        line = line.strip()
        if not line:
            continue
        
        # The issue is in the roll/pitch/yaw columns - they're concatenated
        # Pattern: "...12,3.88-4,-4,-79,-2,7,2" should be "...12,3.88,-4,-4,-79,-2,7,2"
        
        # Find the problematic pattern after SatCount column (position 15)
        parts = line.split(',')
        
        if len(parts) >= 16:
            # Check if roll column (index 15) has the concatenated format
            roll_col = parts[15]
            if '-' in roll_col and not roll_col.startswith('-'):
                # Split on the first non-leading minus sign
                match = re.match(r'([0-9.]+)(-.*)', roll_col)
                if match:
                    roll_val = match.group(1)
                    rest = match.group(2)
                    
                    # Replace the problematic column
                    parts[15] = roll_val
                    
                    # Insert the rest as separate columns
                    remaining_parts = rest.split(',') if ',' in rest else [rest]
                    
                    # Reconstruct the line
                    new_parts = parts[:16] + remaining_parts + parts[16:]
                    line = ','.join(new_parts)
                    fixed_count += 1
        
        fixed_lines.append(line)
    
    print(f"Fixed {fixed_count} corrupted data lines")
    
    # Write to temporary file and load with pandas
    temp_file = csv_path.parent / 'temp_npu_fixed.csv'
    with open(temp_file, 'w') as f:
        for line in fixed_lines:
            f.write(line + '\n')
    
    try:
        # Load the fixed data
        loader = FlightDataLoader()
        data = loader.load_file(temp_file)
        
        # Clean up temp file
        temp_file.unlink()
        
        print(f"✓ NPU data loaded successfully: {len(data)} points")
        return data
        
    except Exception as e:
        print(f"❌ NPU data still has issues after fix: {e}")
        # Clean up temp file
        if temp_file.exists():
            temp_file.unlink()
        
        # Fall back to basic GPS-only loading
        print("Falling back to GPS-only data loading...")
        return load_npu_gps_only(csv_path)


def load_npu_gps_only(csv_path):
    """Load NPU data with only GPS coordinates, ignoring corrupted attitude data."""
    import pandas as pd
    
    try:
        # Read all data first to handle the format issues
        with open(csv_path, 'r') as f:
            lines = f.readlines()
        
        # Parse header
        header = lines[0].strip().split(',')
        
        # Find column indices
        timestamp_idx = header.index('timestamp')
        lat_idx = header.index('latitude') 
        lon_idx = header.index('longitude')
        alt_idx = header.index('gpsAltitude')
        
        # Extract GPS data from each line
        gps_data = []
        for line in lines[1:]:
            if not line.strip():
                continue
            parts = line.strip().split(',')
            
            # Only extract the GPS columns we need
            if len(parts) > max(timestamp_idx, lat_idx, lon_idx, alt_idx):
                try:
                    timestamp = float(parts[timestamp_idx])
                    latitude = float(parts[lat_idx])
                    longitude = float(parts[lon_idx])
                    altitude = float(parts[alt_idx])
                    
                    gps_data.append({
                        'timestamp': timestamp,
                        'latitude': latitude,
                        'longitude': longitude,
                        'gpsAltitude': altitude
                    })
                except (ValueError, IndexError):
                    continue  # Skip corrupted lines
        
        # Create DataFrame
        data = pd.DataFrame(gps_data)
        
        # Rename gpsAltitude to altitude for compatibility
        data['altitude'] = data['gpsAltitude']
        
        # Add dummy attitude data for compatibility
        data['accX'] = 0.0
        data['accY'] = 0.0  
        data['accZ'] = 1.0
        data['gyroX'] = 0.0
        data['gyroY'] = 0.0
        data['gyroZ'] = 0.0
        
        print(f"✓ NPU GPS-only data loaded: {len(data)} points")
        print("⚠️  Using GPS coordinates only (lat, lon, altitude)")
        
        return data
        
    except Exception as e:
        raise Exception(f"Cannot load NPU GPS data: {e}")


class Animated3DFlightViewer:
    """Animated 3D flight path viewer that reveals the path gradually over time."""
    
    def __init__(self, data):
        self.data = data
        self.timestamps = data['timestamp'].values
        # Normalize time to start from 0 seconds
        self.time_normalized = (self.timestamps - self.timestamps[0]) / 1000.0
        
        # Calculate speed between consecutive points
        self.speeds = self._calculate_speeds()
        
        # Fix zero speed values by interpolating from neighbors
        self.speeds = self._fix_zero_speeds(self.speeds)
        
        # Initialize state variables - start in paused state
        self.is_paused = True
        self.manual_frame = None
        self.current_frame = 0
        self.animation_start_frame = 0
        
        # Click interaction for timestamp display (max 3 points)
        self.clicked_points = []  # List of clicked point indices
        self.timestamp_annotations = []  # List of annotation objects
        self.max_annotations = 3
        
        # Maneuver limits planes
        self.maneuver_planes_visible = False
        self.maneuver_planes = []
        
        # Calculate attitude data
        self._calculate_attitudes()
        
        # Calculate dynamic speed scale based on data
        self.max_speed = max(100, np.percentile(self.speeds[self.speeds > 0], 95))  # Use 95th percentile or 100 m/s minimum
        print(f"Dynamic speed scale: 0 to {self.max_speed:.1f} m/s (95th percentile: {np.percentile(self.speeds[self.speeds > 0], 95):.1f})")
        
        # Setup the figure and 3D axis with space for controls
        self.fig = plt.figure(figsize=(12, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Adjust subplot to make room for controls at bottom
        plt.subplots_adjust(bottom=0.25)
        
        # Initialize empty line and point collections
        self.current_point, = self.ax.plot([], [], [], 'ro', markersize=8, label='Current Position')
        self.trail_points = self.ax.scatter([], [], [], c=[], cmap='viridis', s=30, alpha=0.8, label='Speed')
        
        # Animation parameters - optimized for 3x speed and reduced lag
        self.current_frame = 0
        self.frame_skip = 6  # Show every 6th point (3x faster, less data to render)
        self.total_frames = len(data) // self.frame_skip
        self.animation_speed = 8  # milliseconds between frames (2x faster refresh)
        
        # Trail parameters for shorter trace - reduced for better performance
        self.trail_length = 51  # Only show last 51 points in trail
        
        # Setup the plot
        self._setup_plot()
        
        # Setup interactive controls
        self._setup_controls()
        
        # Setup click event handler
        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        
        
    def _calculate_speeds(self):
        """Calculate speed between consecutive GPS points."""
        speeds = np.zeros(len(self.data))
        
        for i in range(1, len(self.data)):
            # Calculate distance between consecutive points (rough approximation)
            lat1, lon1, alt1 = self.data.iloc[i-1][['latitude', 'longitude', 'altitude']]
            lat2, lon2, alt2 = self.data.iloc[i][['latitude', 'longitude', 'altitude']]
            
            # Convert lat/lon differences to meters (rough approximation)
            dlat = (lat2 - lat1) * 111132.92  # meters per degree latitude
            dlon = (lon2 - lon1) * 111412.84 * np.cos(np.radians(lat1))  # meters per degree longitude
            dalt = alt2 - alt1
            
            # 3D distance
            distance = np.sqrt(dlat**2 + dlon**2 + dalt**2)
            
            # Time difference in seconds (using original timestamps for accuracy)
            dt = (self.timestamps[i] - self.timestamps[i-1]) / 1000.0
            
            # Speed in m/s (avoid division by zero)
            if dt > 0:
                speeds[i] = distance / dt
            else:
                speeds[i] = speeds[i-1] if i > 1 else 0
                
        return speeds
        
    def _fix_zero_speeds(self, speeds):
        """Fix zero speed values by interpolating from neighboring non-zero values."""
        speeds_fixed = speeds.copy()
        zero_indices = np.where(speeds == 0)[0]
        
        print(f"Found {len(zero_indices)} zero speed values to fix")
        
        for idx in zero_indices:
            # Find nearest non-zero neighbors
            left_neighbor = None
            right_neighbor = None
            
            # Look for left neighbor
            for i in range(idx - 1, -1, -1):
                if speeds[i] > 0:
                    left_neighbor = speeds[i]
                    break
            
            # Look for right neighbor
            for i in range(idx + 1, len(speeds)):
                if speeds[i] > 0:
                    right_neighbor = speeds[i]
                    break
            
            # Interpolate based on available neighbors
            if left_neighbor is not None and right_neighbor is not None:
                # Use average of both neighbors
                speeds_fixed[idx] = (left_neighbor + right_neighbor) / 2
            elif left_neighbor is not None:
                # Use left neighbor
                speeds_fixed[idx] = left_neighbor
            elif right_neighbor is not None:
                # Use right neighbor
                speeds_fixed[idx] = right_neighbor
            else:
                # No neighbors found, use a small default value
                speeds_fixed[idx] = 1.0
        
        return speeds_fixed
        
    def _calculate_attitudes(self):
        """Calculate roll, pitch, yaw using simple Team 0 configuration (well-aligned datalogger)."""
        print("Calculating attitude data using simple Team 0 configuration...")
        
        # Extract sensor data
        acc_data = np.column_stack([
            self.data['accX'].values,
            self.data['accY'].values, 
            self.data['accZ'].values
        ])
        
        gyro_data = np.column_stack([
            self.data['gyroX'].values,
            self.data['gyroY'].values,
            self.data['gyroZ'].values
        ])
        
        # Calculate time step
        timestamps = self.data['timestamp'].values
        dt = np.mean(np.diff(timestamps)) / 1000.0  # Convert ms to seconds
        
        print(f"Processing {len(acc_data)} samples at {1.0/dt:.1f} Hz...")
        
        # === SIMPLE ATTITUDE CALCULATION ===
        # Basic attitude estimation from accelerometer data
        # Roll: rotation around X-axis (forward), Pitch: rotation around Y-axis (right wing)
        
        # Calculate roll and pitch from accelerometer (assuming static conditions)
        roll_rad = np.arctan2(acc_data[:, 1], acc_data[:, 2])  # atan2(ay, az)
        pitch_rad = np.arctan2(-acc_data[:, 0], np.sqrt(acc_data[:, 1]**2 + acc_data[:, 2]**2))  # atan2(-ax, sqrt(ay²+az²))
        
        # Simple yaw integration from gyroscope (will drift over time)
        yaw_rad = np.zeros_like(roll_rad)
        for i in range(1, len(gyro_data)):
            yaw_rad[i] = yaw_rad[i-1] + gyro_data[i, 2] * dt  # Integrate gz
        
        # Store results (convert to degrees for display)
        self.roll = np.degrees(roll_rad)
        self.pitch = np.degrees(pitch_rad)
        self.yaw = np.degrees(yaw_rad)
        
        # Store same values for backward compatibility
        self.kalman_roll = self.roll
        self.kalman_pitch = self.pitch
        self.kalman_yaw = self.yaw
        
        # === G-FORCE CALCULATION ===
        print("Calculating G-forces...")
        self._calculate_gforces(acc_data)
        
        print(f"\nSimple Attitude Results:")
        print(f"  Roll: {np.min(self.roll):.1f}° to {np.max(self.roll):.1f}° (+ = right wing down)")
        print(f"  Pitch: {np.min(self.pitch):.1f}° to {np.max(self.pitch):.1f}° (+ = nose up)")
        print(f"  Yaw: {np.min(self.yaw):.1f}° to {np.max(self.yaw):.1f}° (+ = clockwise)")
    
    def _setup_controls(self):
        """Setup interactive controls (slider and pause button only)."""
        # Time slider
        ax_slider = plt.axes([0.2, 0.1, 0.5, 0.03])
        self.time_slider = Slider(
            ax_slider, 'Time', 0, self.total_frames - 1, 
            valinit=0, valfmt='%d', valstep=1
        )
        self.time_slider.on_changed(self._on_slider_change)
        
        # Pause/Play button (centered) - start with Play label since we start paused
        pause_ax = plt.axes([0.42, 0.02, 0.16, 0.04])
        self.pause_button = Button(pause_ax, 'Play')
        self.pause_button.on_clicked(self._on_pause_click)
    
    def _on_slider_change(self, val):
        """Handle slider value change."""
        # Avoid recursive updates when animation updates slider
        if hasattr(self, '_updating_slider') and self._updating_slider:
            return
            
        self.manual_frame = int(val)
        self.current_frame = int(val)  # Update current frame to slider position
        
        if self.is_paused:
            # Update display immediately when paused
            self._update_display(self.manual_frame)
            self.fig.canvas.draw()
        else:
            # When playing, reset animation start frame to continue from new position
            self.animation_start_frame = int(val)
    
    def _on_pause_click(self, event):
        """Toggle pause/play."""
        if 'Play' in self.pause_button.label.get_text():
            self.pause_button.label.set_text('Pause')
            self.is_paused = False
            # Resume from current slider position
            self.current_frame = int(self.time_slider.val)
            self.animation_start_frame = self.current_frame
            # Reset the animation to start from the current position
            if hasattr(self, 'anim'):
                self.anim.resume()
        else:
            self.pause_button.label.set_text('Play')
            self.is_paused = True
            # Store current position when pausing
            self.current_frame = int(self.time_slider.val)
            if hasattr(self, 'anim'):
                self.anim.pause()
    
    
    def _on_click(self, event):
        """Handle mouse click events on the 3D plot."""
        if event.inaxes != self.ax:
            return
        
        # Only handle left clicks in the 3D plot area
        if event.button != 1:  # 1 = left click
            return
        
        # Get click coordinates
        click_x, click_y = event.xdata, event.ydata
        if click_x is None or click_y is None:
            return
        
        # Find the closest point in the current trail
        current_frame = int(self.time_slider.val) if hasattr(self, 'time_slider') else 0
        data_index = current_frame * self.frame_skip
        
        if data_index >= len(self.data):
            return
        
        # Get visible trail data
        trail_start = max(0, data_index - self.trail_length)
        trail_indices = range(trail_start, data_index + 1, self.frame_skip)
        
        if len(trail_indices) == 0:
            return
        
        # Find closest point to click
        min_distance = float('inf')
        closest_idx = None
        
        for idx in trail_indices:
            if idx >= len(self.data):
                continue
            
            point_lon = self.data['longitude'].iloc[idx]
            point_lat = self.data['latitude'].iloc[idx]
            
            # Calculate 2D distance (ignoring altitude for easier clicking)
            distance = np.sqrt((point_lon - click_x)**2 + (point_lat - click_y)**2)
            
            if distance < min_distance:
                min_distance = distance
                closest_idx = idx
        
        if closest_idx is not None:
            self._toggle_timestamp_display(closest_idx)
    
    def _toggle_timestamp_display(self, data_idx):
        """Toggle timestamp display for a clicked point (max 3 simultaneous)."""
        # Check if this point is already displayed
        if data_idx in self.clicked_points:
            # Remove this specific annotation
            point_index = self.clicked_points.index(data_idx)
            self.timestamp_annotations[point_index].remove()
            self.clicked_points.pop(point_index)
            self.timestamp_annotations.pop(point_index)
            self.fig.canvas.draw()
            return
        
        # If we have reached max annotations, don't add more
        if len(self.clicked_points) >= self.max_annotations:
            print(f"Maximum {self.max_annotations} points already displayed. Use Clear buttons to remove points.")
            return
        
        # Get point data
        point_lon = self.data['longitude'].iloc[data_idx]
        point_lat = self.data['latitude'].iloc[data_idx]
        point_alt = self.data['altitude'].iloc[data_idx]
        
        # Get original timestamp in seconds
        timestamp_ms = self.data['timestamp'].iloc[data_idx]
        time_seconds = timestamp_ms / 1000.0
        
        # Get additional data for display
        speed = self.speeds[data_idx] if data_idx < len(self.speeds) else 0
        roll = self.roll[data_idx] if data_idx < len(self.roll) else 0
        pitch = self.pitch[data_idx] if data_idx < len(self.pitch) else 0
        gforce = self.gforces[data_idx] if data_idx < len(self.gforces) else 0
        
        # Choose different colors for multiple annotations
        colors = ['white', 'lightblue', 'lightgreen']
        color = colors[len(self.clicked_points) % len(colors)]
        
        # Create annotation text with point number
        point_num = len(self.clicked_points) + 1
        annotation_text = (f"Point {point_num}\n"
                          f"Time: {time_seconds:.0f}s\n"
                          f"Speed: {speed:.1f} m/s\n"
                          f"Alt: {point_alt:.1f}m\n"
                          f"Roll: {roll:.1f}°\n"
                          f"Pitch: {pitch:.1f}°\n"
                          f"G-force: {gforce:.2f}G")
        
        # Add annotation
        annotation = self.ax.text(
            point_lon, point_lat, point_alt,
            annotation_text,
            fontsize=9,
            bbox=dict(boxstyle='round,pad=0.3', facecolor=color, alpha=0.9, edgecolor='black'),
            ha='left', va='bottom'
        )
        
        # Store the new annotation
        self.clicked_points.append(data_idx)
        self.timestamp_annotations.append(annotation)
        
        self.fig.canvas.draw()
    
    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Convert Euler angles to quaternion."""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return np.array([w, x, y, z])
    
    def _calculate_gforces(self, acc_data: np.ndarray):
        """Calculate G-forces from accelerometer data - data is already in G-units."""
        
        # IMPORTANT: The accelerometer data is already in G-units (not m/s²)
        # Evidence: typical values are ~0.07, -0.07, 1.00 which matches G-forces
        
        # Method 1: Total G-force (magnitude including gravity)
        acc_magnitudes = np.linalg.norm(acc_data, axis=1)
        total_gforces = acc_magnitudes  # Already in G-units
        
        # Method 2: Dynamic G-force using attitude compensation
        # Calculate gravity-compensated acceleration using current attitude
        gravity_compensated_acc = []
        
        for i in range(len(acc_data)):
            # Get current attitude (in radians)
            roll = np.radians(self.kalman_roll[i])
            pitch = np.radians(self.kalman_pitch[i])
            
            # Expected gravity vector in body frame (in G-units)
            # Standard aircraft convention: X=forward, Y=right, Z=down
            gravity_body = np.array([
                1.0 * np.sin(pitch),                    # 1G * sin(pitch)
                -1.0 * np.cos(pitch) * np.sin(roll),    # -1G * cos(pitch) * sin(roll)
                -1.0 * np.cos(pitch) * np.cos(roll)     # -1G * cos(pitch) * cos(roll)
            ])
            
            # Remove gravity component from measured acceleration (both in G-units)
            dynamic_acc = acc_data[i] - gravity_body
            gravity_compensated_acc.append(np.linalg.norm(dynamic_acc))
        
        # Use gravity-compensated method as primary
        self.gforces_compensated = np.array(gravity_compensated_acc)
        
        # Method 3: Simple baseline subtraction (|total - 1G|)
        baseline_gforces = np.abs(total_gforces - 1.0)
        
        # Use the gravity-compensated method as primary
        self.gforces = self.gforces_compensated
        
        # Calculate statistics
        self.max_gforce = np.max(self.gforces)
        self.max_gforce_index = np.argmax(self.gforces)
        self.max_gforce_time = self.time_normalized[self.max_gforce_index]
        self.avg_gforce = np.mean(self.gforces)
        self.min_gforce = np.min(self.gforces)
        
        # Find high-G events (>0.5G dynamic is significant for aircraft)
        high_g_indices = np.where(self.gforces > 0.5)[0]
        self.high_g_count = len(high_g_indices)
        
        print(f"\nG-Force Analysis (Data already in G-units):")
        print(f"  Dynamic G-force range: {self.min_gforce:.3f}G to {self.max_gforce:.3f}G")
        print(f"  Average dynamic: {self.avg_gforce:.3f}G")
        print(f"  Maximum: {self.max_gforce:.3f}G at {self.max_gforce_time:.1f}s")
        print(f"  High-G events (>0.5G): {self.high_g_count}")
        print(f"  Total G-force range: {np.min(total_gforces):.3f}G to {np.max(total_gforces):.3f}G")
        print(f"  Baseline method range: {np.min(baseline_gforces):.3f}G to {np.max(baseline_gforces):.3f}G")
        
    def _setup_plot(self):
        """Setup the 3D plot with proper limits and labels."""
        # Use percentiles to ignore outliers while keeping all data points
        # 1st and 99th percentiles will ignore extreme outliers
        lon_p1, lon_p99 = np.percentile(self.data['longitude'], [1, 99])
        lat_p1, lat_p99 = np.percentile(self.data['latitude'], [1, 99])
        alt_p1, alt_p99 = np.percentile(self.data['altitude'], [1, 99])
        
        # Also get min/max for comparison
        lon_min, lon_max = self.data['longitude'].min(), self.data['longitude'].max()
        lat_min, lat_max = self.data['latitude'].min(), self.data['latitude'].max()
        alt_min, alt_max = self.data['altitude'].min(), self.data['altitude'].max()
        
        print(f"Data ranges (min/max): Lat {lat_min:.6f} to {lat_max:.6f}, Lon {lon_min:.6f} to {lon_max:.6f}, Alt {alt_min:.1f} to {alt_max:.1f}")
        print(f"Data ranges (1-99%): Lat {lat_p1:.6f} to {lat_p99:.6f}, Lon {lon_p1:.6f} to {lon_p99:.6f}, Alt {alt_p1:.1f} to {alt_p99:.1f}")
        
        # Add padding to percentile-based ranges
        lon_range = lon_p99 - lon_p1
        lat_range = lat_p99 - lat_p1
        alt_range = alt_p99 - alt_p1
        
        # For very small ranges (like hovering flight), use minimum padding
        min_padding = 0.00001  # Minimum padding in degrees for GPS coordinates
        
        lon_padding = max(lon_range * 0.1, min_padding)
        lat_padding = max(lat_range * 0.1, min_padding)
        alt_padding = max(alt_range * 0.1, 0.5)  # Minimum 0.5m padding for altitude
        
        # Calculate final limits based on percentiles
        lon_limits = (lon_p1 - lon_padding, lon_p99 + lon_padding)
        lat_limits = (lat_p1 - lat_padding, lat_p99 + lat_padding)
        alt_limits = (alt_p1 - alt_padding, alt_p99 + alt_padding)
        
        print(f"Setting axis limits (percentile-based): Lat {lat_limits[0]:.6f} to {lat_limits[1]:.6f}")
        
        # Set plot limits with padding
        self.ax.set_xlim(lon_limits)
        self.ax.set_ylim(lat_limits)
        self.ax.set_zlim(alt_limits)
        
        # Force axis limits to stick
        self.ax.auto_scale_xyz([lon_limits[0], lon_limits[1]], 
                              [lat_limits[0], lat_limits[1]], 
                              [alt_limits[0], alt_limits[1]])
        
        # Labels and title
        self.ax.set_xlabel('Longitude (degrees)')
        self.ax.set_ylabel('Latitude (degrees)')
        self.ax.set_zlabel('Altitude (meters)')
        self.ax.set_title('Animated 3D Flight Path')
        
        # Add legend
        self.ax.legend()
        
        # Add colorbar for speed
        self.colorbar = None
        
        # Add text for current time and speed
        self.time_text = self.ax.text2D(0.02, 0.98, '', transform=self.ax.transAxes, 
                                       fontsize=12, verticalalignment='top',
                                       bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        # Add G-force display box
        self.gforce_text = self.ax.text2D(0.98, 0.25, '', transform=self.ax.transAxes, 
                                         fontsize=10, verticalalignment='bottom', horizontalalignment='right',
                                         bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.9))
        
        
    def animate(self, frame):
        """Animation function called by FuncAnimation."""
        if self.is_paused:
            # When paused, stay at current frame or use manual frame from slider
            if self.manual_frame is not None:
                actual_frame = self.manual_frame
                self.manual_frame = None
            else:
                actual_frame = self.current_frame
        else:
            # When playing, calculate frame based on animation progress from start frame
            actual_frame = (self.animation_start_frame + frame) % self.total_frames
            
            # If we have a manual frame from slider movement, use it and reset animation
            if self.manual_frame is not None:
                actual_frame = self.manual_frame
                self.manual_frame = None
                # Reset animation start frame to continue from this position
                self.animation_start_frame = actual_frame
        
        # Store current frame for pause/resume functionality
        self.current_frame = actual_frame
        
        # Update slider position to match animation (avoid recursive calls)
        if not hasattr(self, '_updating_slider'):
            self._updating_slider = True
            self.time_slider.set_val(actual_frame)
            self._updating_slider = False
        
        return self._update_display(actual_frame)
    
    def _restart_animation_from_frame(self, start_frame):
        """Restart animation from a specific frame."""
        self.animation_start_frame = start_frame
        self.anim.resume()
    
    def _update_display(self, frame):
        """Update the display for a given frame."""
        # Calculate data index from frame
        data_index = frame * self.frame_skip
        
        if data_index >= len(self.data):
            return self.current_point, self.trail_points, self.time_text, self.gforce_text
        
        num_visible = data_index + 1
        
        if num_visible > 0:
            # Get data for shorter trail (only last N points)
            trail_start = max(0, data_index - (self.trail_length * self.frame_skip))
            trail_indices = range(trail_start, data_index + 1, self.frame_skip)
            visible_data = self.data.iloc[trail_indices]
            visible_speeds = self.speeds[trail_indices]
            current_time = self.time_normalized[data_index]
            current_speed = self.speeds[data_index]
            
            # Get current G-force
            current_gforce = self.gforces[data_index]
            
            
            # Update speed-colored trail points - optimized rendering
            if len(visible_data) > 0:
                self.trail_points.remove()
                self.trail_points = self.ax.scatter(visible_data['longitude'], 
                                                   visible_data['latitude'], 
                                                   visible_data['altitude'],
                                                   c=visible_speeds, cmap='viridis', 
                                                   s=20, alpha=0.7, vmin=0, vmax=self.max_speed)  # Smaller points, less alpha
                
                # Add colorbar on first frame
                if self.colorbar is None and len(visible_data) > 1:
                    self.colorbar = plt.colorbar(self.trail_points, ax=self.ax, shrink=0.8, pad=0.1)
                    self.colorbar.set_label('Speed (m/s)')
            
            # Update current position (latest point)
            self.current_point.set_data_3d([self.data['longitude'].iloc[data_index]], 
                                          [self.data['latitude'].iloc[data_index]], 
                                          [self.data['altitude'].iloc[data_index]])
            
            # Update time and speed display
            # Clamp speed to reasonable values to avoid display issues
            display_speed = min(current_speed, 200.0)  # Cap at 200 m/s for display
            self.time_text.set_text(f'Time: {current_time:.0f}s\nSpeed: {display_speed:.1f} m/s\nTrail: {len(visible_data)} pts')
            
            
            # Update G-force display with current and maximum values
            # Highlight if current G-force is high (>0.5G dynamic) or at maximum
            gforce_color = 'red' if current_gforce > 0.5 else 'black'
            max_indicator = ' ⚠️ MAX!' if data_index == self.max_gforce_index else ''
            
            self.gforce_text.set_text(f'G-FORCES (Dynamic)\nCurrent: {current_gforce:.3f}G{max_indicator}\nMaximum: {self.max_gforce:.3f}G\nAt: {self.max_gforce_time:.1f}s\nHigh-G: {self.high_g_count}')
            
        return self.current_point, self.trail_points, self.time_text, self.gforce_text
        
    def start_animation(self):
        """Start the animation."""
        print(f"Starting animation with {self.total_frames} frames...")
        print("Controls:")
        print("  - Time Slider: Drag to jump to any time")
        print("  - Pause/Play: Click to pause/resume animation")
        print("  - Close window to stop")
        print("  - Animation starts PAUSED - press Play to begin")
        
        # Create animation - optimized for performance
        self.anim = animation.FuncAnimation(
            self.fig, self.animate, frames=self.total_frames,
            interval=self.animation_speed, blit=False, repeat=True, cache_frame_data=False
        )
        
        # Start paused and show initial frame
        self.anim.pause()
        self._update_display(0)
        self.fig.canvas.draw()
        
        # Show the plot
        plt.show()
        
        return self.anim


def main():
    """Main entry point for the animated 3D flight viewer."""
    print("=== Animated 3D Flight Viewer ===")
    
    # Determine input file
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        csv_file = "ATA-Flight2.CSV"
        
    csv_path = Path(csv_file)
    if not csv_path.is_absolute():
        csv_path = Path.cwd() / csv_file
    
    # Check if file exists
    if not csv_path.exists():
        print(f"Error: File '{csv_file}' not found.")
        return 1
        
    try:
        # Load flight data
        print(f"\nLoading flight data from: {csv_path}")
        loader = FlightDataLoader()
        
        # Special handling for NPU data with format issues
        if 'NPU' in csv_file.upper():
            print("Detected NPU data - using GPS-only mode...")
            data = load_npu_gps_only(csv_path)
        else:
            data = loader.load_file(csv_path)
        
        # Keep all data points - outliers will be handled by percentile-based axis limits
        print(f"Total data points: {len(data)}")
        
        # Keep full flight data - no trimming
        print(f"Using full flight data: {len(data)} points")
        
        # Create and start the animated viewer
        print(f"\nCreating animated 3D flight viewer...")
        viewer = Animated3DFlightViewer(data)
        
        # Start animation
        anim = viewer.start_animation()
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
        
    return 0


if __name__ == "__main__":
    sys.exit(main())
