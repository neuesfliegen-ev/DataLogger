%% (Function) Offset Remover for all data

function all_sensors = offsetRemoverAllData(all_sensors)
    for i = 1 : numel(all_sensors)
        all_sensors{i}{1} = offsetRemover(all_sensors{i}{1});
    end
end

%% (Function) Offset Remover for a single sensor

function vectorNoOffset = offsetRemover(inVector)
    inVector_mean = mean(inVector);
    vectorNoOffset = inVector - inVector_mean;    
end