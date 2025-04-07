% File name: nexusCSV_to_opensimTRC_MOT.m
%
% Description: convert the .CSV file exported from Vicon Nexus to .TRC and
% file (marker data) and .MOT file (grf data) compatible with OpenSim.
%
% Input: .CSV file, including Bertec GRF data, followed by Vicon marker
% data
%
% Output: .TRC file for experimental marker data storage; .MOT file for GRF
% storage. Both can be directly read by OpenSim.
% [tested successfully on OpenSim 4.5]
%
% Version history:
% 20250306: basic function: split .CSV file; change axis definition; add
% necessary format details; export to .TRC file.

% TYPE .CSV FILE PATH & OUTPUT FILE PATH HERE
fileName = "F:\Downloads\Stepping.csv";
outputName = "markerData\Vicon Test 0304\Stepping_new.trc";

import org.opensim.modeling.*

%% split the file into two files (GRF & marker data)
fid = fopen(fileName, 'r');

% step A: find the demacration line
lineCounter = 0;
lineVerifier = [];
while ~feof(fid)
    % read in line
    line = fgetl(fid);
    lineCounter = lineCounter + 1;

    % derive the string at the first column
    tokens = split(line, ',');
    firstColumn = strtrim(tokens{1}); % delete possible blank spaces at head & tail

    % string matching
    if strcmp(firstColumn, 'Trajectories')
        lineVerifier = lineCounter;
        break;
    end
end
fclose(fid);

% step B: verification
if isempty(lineVerifier)
    error('Can not find the title of the marker data table.');
end

% step C: read the marker data table (second one)
opts = detectImportOptions(fileName);
opts.DataLines = [lineCounter+5 Inf]; % data line
opts.VariableNamesLine = lineCounter + 2; % marker name line
opts.VariableUnitsLine = lineCounter + 4; % data in mm or m
markerData = readtable(fileName, opts);

clear fid lineCounter lineVerifier fileName firstColumn opts line tokens

%% read data into OpenSim TRC file

dataMatrix = nan(size(markerData));
for i = 1 : width(markerData)
    switch class(markerData{1,i})
        case 'cell'
            tempCol = markerData{:,i};
            for j = 1 : height(markerData)
                tempData = markerData(j,i).Variables;
                dataMatrix(j,i) = str2double(tempData{1});
            end
        case 'double'
            dataMatrix(:,i) = markerData{:,i};
        otherwise
            error(['detected invalid data input at column ' num2str(i) '.']);
    end
end

time = (markerData.Var1-1) / 100; % 100 Hz
dataMatrix(:,2) = time;
markerNames = markerData.Properties.VariableDescriptions;
if isempty(markerNames)
    markerNames = markerData.Properties.VariableNames;
end

fid = fopen(outputName, 'w');

% first row
fprintf(fid, 'PathFileType\t%d\t(X/Y/Z)\t%s\n', 4, outputName);

% second row
fprintf(fid, 'DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\tOrigDataRate\tOrigDataStartFrame\tOrigNumFrames\n');

% third row
fprintf(fid, '%d\t%d\t%d\t%d\tmm\t%d\t%d\t%d\n', ...
    100,100,height(dataMatrix),(width(dataMatrix)-2)/3, ...
    100,dataMatrix(1,1),height(dataMatrix));

% fourth row: marker names
fprintf(fid, 'Frame#\tTime');
for i = 3 : 3 : length(markerNames)
    temp = split(markerNames{i},':');
    if length(temp) == 2
        temp = temp{2};
    elseif isscalar(temp)
        temp = temp{1};
    end
    fprintf(fid, '\t%s\t\t', temp);
end
fprintf(fid, '\n');

% fifth row: X/Y/Z specifications
fprintf(fid, '\t');
for i = 3 : 3 : length(markerNames)
    fprintf(fid, '\tX%d\tY%d\tZ%d', floor(i/3), floor(i/3), floor(i/3));
end
fprintf(fid, '\n');

% sixth row: blank
fprintf(fid, '\n');

% data writing
for i = 1 : height(dataMatrix)
    % Frame# | Time
    for j = 1 : 2
        fprintf(fid, '%d\t', dataMatrix(i,j));
    end
    for j = 1 : (width(dataMatrix)-2)/3
        fprintf(fid, '%d\t%d\t%d\t', dataMatrix(i,3*j), dataMatrix(i,3*j+2), -dataMatrix(i,3*j+1));
    end
    fprintf(fid, '\n');
end


fclose(fid);