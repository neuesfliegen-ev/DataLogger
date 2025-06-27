function all_sensors = uniformAllData(time_uni, time_s, all_sensors)

    for i = 1 : numel(all_sensors)
        all_sensors{i}{1} = interp1(  ...
             time_s,                  ...% original timestamps
             all_sensors{i}{1},       ...% your altitude vector (DC removed)
             time_uni,                ...% uniform timestamps
             'linear');               ...% interpolation method
    end
    
end