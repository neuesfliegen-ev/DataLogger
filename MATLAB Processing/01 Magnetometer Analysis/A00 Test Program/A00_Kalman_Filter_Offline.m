%% Purpose of the program: Visualize raw magnetomer data in µT and apply an offline Extended Kalman Filter to calibrate the magnetic field data of a plane.

%% Multiple files
NUMBER_OF_FILES = 5;
colors = lines(NUMBER_OF_FILES);

% Clears out the screen and figures
clc;
%close all;

% Preallocate cell array to hold all imported files
allData = cell(NUMBER_OF_FILES,1);

% Iterate the reading process
for i = 0:(NUMBER_OF_FILES - 1)
    % Build filename with zero-padded index
    fname = sprintf("testingDoc%d_sensor_mag_0.csv", i+1);
    
    % Read table, first row is the header
    T = readtable(fname, "HeaderLines", 0);
    
    % Store in cell array
    allData{i+1} = T;
    
    % % (Optional) Display first 5×5 block to verify
    % fprintf("First 5×5 of %s:\n", fname);
    % disp(T(1:min(5,height(T)), 1:min(5,width(T))));
end

%% Full plot of each example

for i = 1:(NUMBER_OF_FILES)
    txt = sprintf('Magnetometer Example %d', i);
    figure('Name', txt);
    tiledlayout(2,2)

    T = allData{i};

    % Magnetometer
    x = T{:,4};
    y = T{:,5};
    z = T{:,6};    

    % Time
    time_us = T{:,1};
    time_us = time_us - time_us(1);
    time_s = time_us * 10^6; 

    %% Raw Data
    nexttile;
    plot(time_s, x, 'Color', colors(1,:), 'LineWidth',0.6, 'MarkerSize',5);
    hold on;
    plot(time_s, y, 'Color', colors(2,:), 'LineWidth',0.6, 'MarkerSize',5);
    hold on;
    plot(time_s, z, 'Color', colors(3,:), 'LineWidth',0.6, 'MarkerSize',5);
    hold on;
    ylabel('Magnetometer (µT)');
    xlabel('Time (s)');
    hold off;
    title("Raw Data");
    legend({'magX','magY','magZ'}, 'Location','best');
    
    %% Raw 3D
    nexttile(3);
    scatter3(x, y, z, 20, 'b', '.');   % 5-point size, dot marker
    axis equal;                  % ensure X, Y, Z axes use the same scale
    grid on;
    xlabel('X (µT)');
    ylabel('Y (µT)');
    zlabel('Z (µT)');
    title("Raw 3D Representations");

    %% Kalman Filter
 
    % 1) Fit the ellipsoid
    [b, C] = A01_ellipsoidFit(x, y, z);

    fprintf("Estimated hard-iron offset for example %d (µT): [%.2f, %.2f, %.2f]\n", i, b);
    disp("Soft-iron matrix C:");
    disp(C);

    % 2) Apply calibration
    Mraw = [x, y, z]';
    Mcorr = C * (Mraw - b);

    Xc = Mcorr(1,:)'; 
    Yc = Mcorr(2,:)'; 
    Zc = Mcorr(3,:)';

    nexttile(4);
    scatter3(Xc, Yc, Zc, 20, 'r', '.'); 
    axis equal; 
    title("Calibrated 3D Representations");
    xlabel("X (µT)"); 
    ylabel("Y (µT)"); 
    zlabel("Z (µT)"); 
    grid on;

    %% Table with Kalman Filter information
    nexttile(2);

    axis off

    % build the two lines of text
    msg1 = sprintf("Hard-iron offset (µT): [%.2f, %.2f, %.2f]", b(1), b(2), b(3));
    % format the 3×3 matrix on one or more lines
    msg2 = sprintf("Soft-iron C:\n  [%.3f  %.3f  %.3f]\n  [%.3f  %.3f  %.3f]\n  [%.3f  %.3f  %.3f]", ...
                   C(1,1),C(1,2),C(1,3), ...
                   C(2,1),C(2,2),C(2,3), ...
                   C(3,1),C(3,2),C(3,3));
    
    % place the text centered in that tile
    text(0.5, 0.7, msg1, 'Units','normalized', ...
         'HorizontalAlignment','center','FontSize',10);
    text(0.5, 0.3, msg2, 'Units','normalized', ...
         'HorizontalAlignment','center','FontSize',10);

    title("Calibration Points");
end

