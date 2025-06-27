%% (Function) Plots All Data

function plotAllData(time, all_sensors, color)
    figure                              % open one figure window
    tiledlayout(3,2)                    % split into 2 rows, 2 columns

    for i = 1 : 6
        plotData(time, all_sensors{i}{1}, all_sensors{i}{3}, color);
    end

    figure                             
    tiledlayout(3,2)

    for i = 7 : 12
        plotData(time, all_sensors{i}{1}, all_sensors{i}{3}, color);
    end

    figure                              
    tiledlayout(1,2) 

    for i = 13 : 14
        plotData(time, all_sensors{i}{1}, all_sensors{i}{3}, color);
    end

    figure                              
    tiledlayout(3,2)

    for i = 15 : 20
        plotData(time, all_sensors{i}{1}, all_sensors{i}{3}, color);
    end
end

%% (Function) Plots Data from a single sensor

function plotData(timeVector, inVector, nameVariable, color)
    nexttile;
    plot(timeVector, inVector, '.-', 'LineWidth', 0.6, 'MarkerSize', 5, 'Color', color);
    xlabel('Time [s]');
    ylabel('Magnitude');
    grid on;
    title(nameVariable);
end
