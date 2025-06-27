%% (Function) Plots All Data

function plotAllDataFourier(all_sensors, color, N, Fs)
    figure                              % open one figure window
    tiledlayout(3,2)                    % split into 2 rows, 2 columns

    for i = 1 : 6
        plotDataFourier(all_sensors{i}{1}, all_sensors{i}{3}, color, N, Fs);
    end

    figure                             
    tiledlayout(3,2)

    for i = 7 : 12
        plotDataFourier(all_sensors{i}{1}, all_sensors{i}{3}, color, N, Fs);
    end

    figure                              
    tiledlayout(1,2) 

    for i = 13 : 14
        plotDataFourier(all_sensors{i}{1}, all_sensors{i}{3}, color, N, Fs);
    end

    figure                              
    tiledlayout(3,2)

    for i = 15 : 20
        plotDataFourier(all_sensors{i}{1}, all_sensors{i}{3}, color, N, Fs);
    end
end

%% (Function) Plots Data from a single sensor

function plotDataFourier(inVector, nameVariable, color, N, Fs)
    
    % % Plotting of the raw FFT
    % title1 = sprintf('%s: %s', nameVariable, "Complex Magnitude of FFT Spectrum"); 
    % 
    % nexttile;
    % plot(Fs/N*(0:N-1),abs(inVector), '.-', 'LineWidth', 0.6, 'MarkerSize', 5, 'Color', color);
    % xlabel("f (Hz)")
    % ylabel("|fft(X)|")
    % grid on
    % title(title1)

    % Double-sided FFT Spectrum
    title1 = sprintf('%s: %s', nameVariable, "FFT Spectrum"); 

    nexttile;
    plot(Fs/N*(-N/2:N/2-1), abs(fftshift(inVector)), '.-', 'LineWidth', 0.6, 'MarkerSize', 5, 'Color', color);
    xlabel("f (Hz)")
    ylabel("|fft(X)|")
    grid on
    title(title1)

    % % Single-sided FFT Spectrum
    % P2 = abs(inVector/N);
    % P1 = P2(1:N/2+1);
    % P1(2:end-1) = 2*P1(2:end-1);
    % 
    % f = Fs/N*(0:(N/2));
    % 
    % title1 = sprintf('%s: %s', nameVariable, "Single-sided FFT Spectrum"); 
    % 
    % nexttile;
    % plot(f, P1, '.-', 'LineWidth', 0.6, 'MarkerSize', 5, 'Color', color);
    % xlabel("f (Hz)")
    % ylabel("|P1(f)|")
    % grid on
    % title(title1)

end
