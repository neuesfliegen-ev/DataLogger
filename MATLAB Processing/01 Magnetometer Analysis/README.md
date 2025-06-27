# Application of a Kalman Filter for robustness and precision

The following program can be obtained running "A00_Kalman_Filter_Offline.m". It includes hard-iron and soft-iron corrections, and accounts for severe magnetic field spikes possibly caused by temporary surrounding objects.

The data used to make the analysis is transferred directly from the USB powering the Arduino, and captured through a simple Windows PowerShell program, "00_ReadUSB_toTxt.ps1". The .ino file is also adapted for this purpose and all necessary files can be found in the folder "A02 General Purpose Programs".

![Magnetometer](KalmanFilter.png)
