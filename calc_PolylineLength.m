function totalLength = calc_PolylineLength(location)
% Input：
% - location: 3×n matrix，representing 3-D positions of n points
% Output：
% - totalLength: total length of the polyline

% check if the input is 3xn matrix
if size(location, 1) ~= 3
    error('input location must be the format of 3xn.');
end

% calculate differential positions of adjacent points
diffPoints = diff(location, 1, 2);  % diff along column

% calculate Euclidean distances of each segment
segmentLengths = sqrt(sum(diffPoints.^2, 1));  % square -> sum -> sqrt

% total length
totalLength = sum(segmentLengths);
end
