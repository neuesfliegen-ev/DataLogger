%% Purpose of the program: Read magnetometer data from a limited .txt file and process it through a Kalman Filter

function magTbl = A02_readTxt_mag(filename)

    % -------- 1. read everything (header + numbers, commas as delimiter)
    magTbl = readtable(filename,             ...
                   'Delimiter'      , ',',    ...  % comma-separated
                   'NumHeaderLines' , 9  ,    ...  % skip 9 rows, header is line 10
                   'EmptyLineRule'  , 'read', ...  % keep place for blanks
                   'MissingRule'    , 'omitrow');  % throw away rows that are all NaN

    % % -------- 2. drop empty rows that became NaN after readmatrix
    % magTbl = magRaw(~any(isnan(magRaw),2), :);

    % Check
    disp(magTbl(1:3,:))     % shows the first three rows
end

