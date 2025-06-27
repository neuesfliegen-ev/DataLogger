function all_sensors = fourierAllData(all_sensors, N)
    for i = 1 : numel(all_sensors)
        all_sensors{i}{1} = fft(all_sensors{i}{1}, N);
    end
end