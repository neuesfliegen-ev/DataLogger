%% Purpose of the program: Apply a simpler calibration than the implemented in "00 Ideal Spherical Calibration" accounting only for the hard-iron offsets.

% 1.  Accept a list of files:
filename = "nano_datalog_2Axis_calibration.txt";
colors   = lines(3);

%% Find the first and last lines of data in the .txt file after calibration. The purpose is to have initialization data, and already calibrated data in separate tables for analysis.

lines = readlines(filename);                                % 1-by-N string array, one element per line
firstRow = find(contains(lines, "magX,magY,magZ"));         % Creates a vector with all matching positions firstRow = [7 1200]
lastRow  = find(contains(lines, "Stopped recording data."));

%% Import data
% Used for finding the offsets
T       = A01_readTxt_mag(filename, firstRow(1), lastRow(1));
x_calib = T{:,1};   y_calib = T{:,2};   z_calib = T{:,3};

% Used to apply those offsets
T2     = A01_readTxt_mag(filename, firstRow(2), lastRow(2));
x_data = T2{:,1};   y_data = T2{:,2};   z_data = T2{:,3};

%% Find calibration offsets

bx = 0.5 * (max(x_calib) + min(x_calib));
by = 0.5 * (max(y_calib) + min(y_calib));
bz = 0.5 * (max(z_calib) + min(z_calib));

Xc = x_data - bx;   Yc = y_data - by;   Zc = z_data - bz;

%% Plotting
figure('Name',sprintf('Magnetometer analysis'));
tiledlayout(2,2)

nexttile
plot([x_data y_data z_data]);          % three coloured lines automatically
xlabel('Time (s)')
ylabel('Magnetometer (\muT)')
legend('magX','magY','magZ','Location','best')
title('Raw data')

nexttile(3)
scatter3(x_data,y_data,z_data,6,'filled')  % smaller marker for large files
axis equal, grid on
title('Raw 3-D Representations')

nexttile(4)
scatter3(Xc,Yc,Zc,6,'filled','MarkerFaceColor',[1 0 0])
axis equal, grid on
title('Calibrated 3-D Representations')

nexttile(2)
axis off
txt = sprintf(['Hard-iron b (\\muT):\n' ...
               '[%.2f %.2f %.2f]\n'], bx,by,bz);
text(0.5,0.5,txt,'Units','normalized','HorizontalAlignment','center')

