%% (Function) Reads Data from a single sensor

function outVector = readData(columnLetter)
    rng = sprintf('%s:%s', columnLetter, columnLetter);
    
    outVector = readmatrix( ...
        'Datalogger.xlsx',  ...
        'Range',rng,        ...      % second specific column
        'OutputType','double');

    outVector = outVector(2:end);
end