%% Purpose of the program: Read magnetometer data from a limited .txt file and process it through a Kalman Filter

function magTbl = A01_readTxt_mag(filename, firstRow, lastRow)
    
    rowRange = sprintf('%d:%d', firstRow + 2, lastRow);

    % -------- 1. read everything (header + numbers, commas as delimiter)
    magTbl = readtable(filename,                                    ...
                   'Delimiter'      , ',',                          ...  % comma-separated
                   'Range'          , rowRange,                     ...  % read firstRow → lastRow only
                   'EmptyLineRule'  , 'read',                       ...  % keep place for blanks
                   'MissingRule'    , 'omitrow');                        % throw away rows that are all NaN

    % % -------- 2. drop empty rows that became NaN after readmatrix
    % magTbl = magRaw(~any(isnan(magRaw),2), :);

    % Check
    disp(magTbl(1:3,:))     % shows the first three rows
end

