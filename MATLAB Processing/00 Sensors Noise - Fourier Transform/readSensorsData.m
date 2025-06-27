%% (Function) Reads Data from all sensors

% Output is allSensors. Input is allSensors. Confusing, but otherwise my variable just dies locally.
function all_sensors = readSensorsData(all_sensors)
    for i = 1 : numel(all_sensors)
        all_sensors{i}{1} = readData(all_sensors{i}{2});
    end
end