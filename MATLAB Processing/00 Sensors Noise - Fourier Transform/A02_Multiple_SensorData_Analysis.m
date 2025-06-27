% Purpose of the program: Merge the previous programs A00_SensorData_Plots & A01_Single_SensorData_Analysis to iterate through all sensors.

%% Definitions & Data reading

% Define cells for every sensor { [data array], excelColumn, name }
time_s_info         = { [],   'A', 'Time (s)' };
accelerometerX_info = { [],   'B', 'Accelerometer X' };  accelerometerY_info = { [],   'C', 'Accelerometer Y' };  accelerometerZ_info = { [],   'D', 'Accelerometer Z' };
gyroscopeX_info     = { [],   'E', 'Gyroscope X'     };  gyroscopeY_info     = { [],   'F', 'Gyroscope Y'     };  gyroscopeZ_info     = { [],   'G', 'Gyroscope Z'     };
magnetometerX_info  = { [],   'H', 'Magnetometer X'  };  magnetometerY_info  = { [],   'I', 'Magnetometer Y'  };  magnetometerZ_info  = { [],   'J', 'Magnetometer Z'  };
latitude_info       = { [],   'K', 'Latitude'        };  altitude_info       = { [],   'L', 'Altitude'        };  gpsAltitude_info    = { [],   'M', 'GPS Altitude'    };
speed_info          = { [],   'N', 'Speed'           };  satCount_info       = { [],   'O', 'Satellite Count' };
roll_info           = { [],   'P', 'Roll'            };  pitch_info          = { [],   'Q', 'Pitch'           };  yaw_info            = { [],   'R', 'Yaw'            }; 
pressure_info       = { [],   'S', 'Pressure'        };  temperature_info    = { [],   'T', 'Temperature'     };  pAltitude_info      = { [],   'U', 'Pressure Altitude' };


% Define a cell containing all sensors
allSensors = {
    accelerometerX_info,  accelerometerY_info,  accelerometerZ_info,  ...
    gyroscopeX_info,    gyroscopeY_info,    gyroscopeZ_info,    ...
    magnetometerX_info, magnetometerY_info, magnetometerZ_info, ...
    latitude_info,      altitude_info,      gpsAltitude_info,   ...
    speed_info,         satCount_info,                          ...
    roll_info,          pitch_info,         yaw_info,           ...
    pressure_info,      temperature_info,   pAltitude_info      ...
};

% READ Time
time_ms = readData('A');
time_ms = time_ms - time_ms(1);                 % adjust the reference
time_hs = time_ms / 1000 / 60 / 60;             % time in hours
time_s = time_ms / 1000;                        % time in seconds

% READ Data from all sensors
allSensors = readSensorsData(allSensors);

%% Plotting Original & No-offset Signals

clc                                             % Clears Command Window
plotAllData(time_s, allSensors, 'b');

allSensors2 = offsetRemoverAllData(allSensors);
% plotAllData(time_s, allSensors2, 'r');        % Commented out to not overcrowd the screen.

%% Data Pre-Processing

% We need a uniform sampling for the FFT (our current sampling goes around 1995 - 2010 points per row. We need this fixed).
delta_t  = time_s(2) - time_s(1);        % seconds
t_start  = time_s(1);                    % first timestamp
t_end    = time_s(end);                  % last timestamp
t_uni    = t_start : delta_t : t_end;    % uniform grid in seconds

allSensors3 = uniformAllData(t_uni, time_s, allSensors2);
% plotAllData(t_uni, allSensors3, 'black');     % Commented out to not overcrowd the screen.

%% Fast Fourier Transform

N   = numel(allSensors3{1}{1});         % Number of points (lenght of the signal in terms of vector size)
Fs  = 1 / delta_t;                      % 1 / seconds between each reading. Sampling frequency in Hz (fs = 0.5 for 2-s spacing)

% Compute the Fourier transform of the signal (Complex spectrum, real and imaginary)
allSensorsFourier = fourierAllData(allSensors3, N);
plotAllDataFourier(allSensorsFourier, 'm', N, Fs);