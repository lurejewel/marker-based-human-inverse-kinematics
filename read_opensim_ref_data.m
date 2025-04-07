function [data, labels] = read_opensim_ref_data(filepath)
import org.opensim.modeling.*
file = Storage(filepath);
colNum = file.getColumnLabels.getSize-1; % excluding the time column

% store data
data = nan(file.getSize, colNum);
for i = 1 : file.getSize
    
    stateVector = file.getStateVector(i-1);
    for j = 1 : colNum
        data(i,j) = stateVector.getData.get(j-1);
    end
    
end

% store labels
labels = cell(colNum, 1);
for i = 1 : colNum

    labels{i} = char(file.getColumnLabels.get(i));

end
