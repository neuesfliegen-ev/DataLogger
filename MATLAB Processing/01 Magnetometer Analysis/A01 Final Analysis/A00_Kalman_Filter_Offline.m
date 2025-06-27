%% Purpose of the program: Visualize raw magnetomer data in µT and apply an offline Extended Kalman Filter to calibrate the magnetic field data of a plane.

% 1.  Accept a list of files:
filename = "nano_datalog_mag2.txt";
colors    = lines(3);

T      = A02_readTxt_mag(filename);
x      = T{:,1};   y = T{:,2};   z = T{:,3};

% ---------- robust ellipsoid fit ---------------------------------------
maxIter = 5;
keep    = true(size(x));                 % start with everything

for k = 1:maxIter
    [b,C] = A01_ellipsoidFit( x(keep), y(keep), z(keep) );

    % Mahalanobis distance in the calibrated space
    Mraw  = [x y z]';
    Mcorr = C * (Mraw - b);
    d     = vecnorm(Mcorr)';             % distance from centre

    % compute median + 3×MAD of distances
    dMed  = median(d(keep));
    dMad  = mad(d(keep),1);

    newKeep = d < dMed + 3*dMad;         % inside the 3-MAD shell

    % stop if nothing changes
    if all(newKeep == keep), break, end
    keep = newKeep;
end
% ------------------------------------------------------------------------
% Now I create the new calibrated vectors discarding the spikes
x_calib = x(keep);  y_calib = y(keep);  z_calib = z(keep);

t_us   = (0:2:2*(height(T)-1));     % because I know that my timestamp is 2 micro seconds.
t_s    = t_us * 1e-6;

figure('Name',sprintf('Magnetometer example'));
tiledlayout(2,2)

nexttile
plot([x y z]);          % three coloured lines automatically
xlabel('Time (s)')
ylabel('Magnetometer (\muT)')
legend('magX','magY','magZ','Location','best')
title('Raw data')

nexttile(3)
scatter3(x,y,z,6,'filled')  % smaller marker for large files
axis equal, grid on
title('Raw 3-D Representations')

[b,C] = A01_ellipsoidFit(x_calib,y_calib,z_calib);
Bmag = median( vecnorm(Mcorr) );                  % Median of |B| over all samples
Mcorr = C*( [x_calib y_calib z_calib]' - b );     % Normalized magnetometer values (plots a range of -1 to 1)
Xc = Mcorr(1,:).';  Yc = Mcorr(2,:).';  Zc = Mcorr(3,:).';

nexttile(4)
scatter3(Xc,Yc,Zc,6,'filled','MarkerFaceColor',[1 0 0])
axis equal, grid on
title('Calibrated 3-D Representations')

nexttile(2)
axis off
txt = sprintf(['Hard-iron (\\muT):\n' ...
               '[%.2f %.2f %.2f]\n',...
               'Soft-iron C:\n' ...
               '[% .3f % .3f % .3f]\n' ...
               '[% .3f % .3f % .3f]\n' ...
               '[% .3f % .3f % .3f]' ],...
               b(1),b(2),b(3),C');
text(0.5,0.5,txt,'Units','normalized','HorizontalAlignment','center')

