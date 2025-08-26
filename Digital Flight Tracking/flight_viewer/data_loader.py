import pandas as pd
import numpy as np
from pathlib import Path


class FlightDataLoader:
    """Loads flight data from CSV or Excel files and extracts required columns."""
    
    def __init__(self):
        self.required_columns = ['timestamp', 'latitude', 'longitude', 'altitude']
        self.column_mappings = {
            'altitude': ['altitude', 'gpsAltitude', 'alt', 'height']
        }
        self.data = None
        
    def load_file(self, file_path):
        """
        Load flight data from CSV or Excel file.
        
        Args:
            file_path (str): Path to the data file
            
        Returns:
            pd.DataFrame: DataFrame with timestamp, latitude, longitude, altitude columns
        """
        file_path = Path(file_path)
        
        if not file_path.exists():
            raise FileNotFoundError(f"File not found: {file_path}")
            
        # Determine file type and load accordingly
        if file_path.suffix.lower() == '.xlsx':
            df = pd.read_excel(file_path)
        elif file_path.suffix.lower() == '.csv':
            df = pd.read_csv(file_path)
        else:
            raise ValueError(f"Unsupported file format: {file_path.suffix}")
            
        # Map column names to handle variations
        mapped_columns = {}
        for required_col in self.required_columns:
            found_col = None
            
            # Check for exact match first
            if required_col in df.columns:
                found_col = required_col
            # Check for alternative names
            elif required_col in self.column_mappings:
                for alt_name in self.column_mappings[required_col]:
                    if alt_name in df.columns:
                        found_col = alt_name
                        break
            
            if found_col is None:
                print(f"Available columns: {list(df.columns)}")
                alternatives = self.column_mappings.get(required_col, [required_col])
                raise ValueError(f"Missing required column '{required_col}'. Looked for: {alternatives}")
            
            mapped_columns[required_col] = found_col
            
        print(f"Column mapping: {mapped_columns}")
        
        # Extract and rename columns
        selected_data = {}
        for standard_name, actual_name in mapped_columns.items():
            selected_data[standard_name] = df[actual_name]
            
        # Also include all other columns that might be useful (IMU data, etc.)
        for col in df.columns:
            if col not in mapped_columns.values():
                selected_data[col] = df[col]
            
        self.data = pd.DataFrame(selected_data)
        
        # Keep timestamps as numeric values (milliseconds) - don't convert to datetime
        # This avoids the 1970 epoch issue and keeps them as simple numbers
        if pd.api.types.is_datetime64_any_dtype(self.data['timestamp']):
            # If somehow they got converted to datetime, convert back to numeric
            self.data['timestamp'] = self.data['timestamp'].astype('int64') // 1000000
        # Ensure timestamps are numeric
        self.data['timestamp'] = pd.to_numeric(self.data['timestamp'], errors='coerce')
                
        # Remove any rows with NaN values in critical columns
        initial_rows = len(self.data)
        self.data = self.data.dropna()
        if len(self.data) < initial_rows:
            print(f"Removed {initial_rows - len(self.data)} rows with missing data")
            
        print(f"Loaded {len(self.data)} data points from {file_path}")
        print(f"Time range: {self.data['timestamp'].min()} to {self.data['timestamp'].max()}")
        
        return self.data
        
    def get_data(self):
        """Return the loaded data."""
        if self.data is None:
            raise ValueError("No data loaded. Call load_file() first.")
        return self.data
        
    def get_summary(self):
        """Print summary statistics of the loaded data."""
        if self.data is None:
            print("No data loaded.")
            return
            
        print("\n=== Flight Data Summary ===")
        print(f"Total samples: {len(self.data)}")
        print(f"Latitude range: {self.data['latitude'].min():.6f} to {self.data['latitude'].max():.6f}")
        print(f"Longitude range: {self.data['longitude'].min():.6f} to {self.data['longitude'].max():.6f}")
        print(f"Altitude range: {self.data['altitude'].min():.1f} to {self.data['altitude'].max():.1f} meters")
        
        if pd.api.types.is_datetime64_any_dtype(self.data['timestamp']):
            duration = (self.data['timestamp'].max() - self.data['timestamp'].min()).total_seconds()
            print(f"Flight duration: {duration:.1f} seconds ({duration/60:.1f} minutes)")
