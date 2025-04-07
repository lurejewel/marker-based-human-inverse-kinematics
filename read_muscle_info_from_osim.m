function [muscleArr, muscleMap] = read_muscle_info_from_osim(filePath)

% read .OSIM file
doc = xmlread(filePath);

% find all Thelen2003Muscle elements
muscles = doc.getElementsByTagName('Thelen2003Muscle');

% init muscle name cells
muscleNames = cell(1, muscles.getLength);

% for every <Thelen2003Muscle> elements
for i = 0:muscles.getLength-1

    muscle = muscles.item(i);

    % basic information about the muslce
    muscleName = char(muscle.getAttribute('name')); % name
    muscleMVC = str2double(muscle.getElementsByTagName('max_isometric_force').item(0).getTextContent()); % max isometric force / max voluntary contraction
    muscleOptFiberLen = str2double(muscle.getElementsByTagName('optimal_fiber_length').item(0).getTextContent()); % optimal fiber length
    muscleTendonSlackLen = str2double(muscle.getElementsByTagName('tendon_slack_length').item(0).getTextContent()); % tendon slack length (when no force is loaded)
    musclePenAngleAtOPt = str2double(muscle.getElementsByTagName('pennation_angle_at_optimal').item(0).getTextContent()); % pennation angle (angle between tendon and fiber) at optimal fiber length (in rad)
    muscleFmaxTendonStrain = str2double(muscle.getElementsByTagName('FmaxTendonStrain').item(0).getTextContent()); % tendon strain at mvc
    muscleFmaxMuscleStrain = str2double(muscle.getElementsByTagName('FmaxMuscleStrain').item(0).getTextContent()); % passive muscle strain at mvc
    muscleKshapeActive = str2double(muscle.getElementsByTagName('KshapeActive').item(0).getTextContent()); % shape factor for Gaussian active muscle force-length relationship
    muscleKshapePassive = str2double(muscle.getElementsByTagName('KshapePassive').item(0).getTextContent()); % exponential shape factor for passive force-length relationship
    muscleAf = str2double(muscle.getElementsByTagName('Af').item(0).getTextContent()); % force-velocity shape factor
    muscleFlen = str2double(muscle.getElementsByTagName('Flen').item(0).getTextContent()); % max normalized lengthening force
    muscleActTime = str2double(muscle.getElementsByTagName('activation_time_constant').item(0).getTextContent()); % activation time constant (in sec)
    muscleDeactTime = str2double(muscle.getElementsByTagName('deactivation_time_constant').item(0).getTextContent()); % deactivation time constatn (in sec)

    % init MuscleDef class
    muscleNames{i+1} = muscleName;
    muscleArr(i+1) = MuscleDef('name', muscleName, ...
        'mvc', muscleMVC, ...
        'optFiberLen', muscleOptFiberLen, ...
        'tendonSlackLen', muscleTendonSlackLen, ...
        'penAngleAtOpt', musclePenAngleAtOPt, ...
        'FmaxTendonStrain', muscleFmaxTendonStrain, ...
        'FmaxMuscleStrain', muscleFmaxMuscleStrain, ...
        'KshapeActive', muscleKshapeActive, ...
        'KshapePassive', muscleKshapePassive, ...
        'Af', muscleAf, ...
        'Flen', muscleFlen, ...
        'actTime', muscleActTime, ...
        'deactTime', muscleDeactTime);

    %%% --- GET PATH POINTS OF THE MUSCLE --- %%%
    pathPointSet = muscle.getElementsByTagName('PathPointSet').item(0);

    % 1. General path points
    pathPoints = pathPointSet.getElementsByTagName('PathPoint');

    % for each path point of the muscle
    for j = 0:pathPoints.getLength-1
        pathPoint = pathPoints.item(j);

        % get socket_parent_frame and location of each path point
        socketParent = char(pathPoint.getElementsByTagName('socket_parent_frame').item(0).getTextContent());
        socketParent = extractAfter(socketParent, '/bodyset/');
        location = char(pathPoint.getElementsByTagName('location').item(0).getTextContent());

        % stored into MuclePathPointDef class
        p = MusclePathPointDef('attachedBody', socketParent, 'location', str2num(location)); %#ok<ST2NM>
        muscleArr(i+1) = muscleArr(i+1).add_musclePathPoint(p);
    end

    % 2. Conditional path points
    condPathPoints = pathPointSet.getElementsByTagName('ConditionalPathPoint');

    % for each path point of the muscle
    for j = 0:condPathPoints.getLength-1
        condPathPoint = condPathPoints.item(j);

        % get socket_parent_frame, location and range of each path point
        socketParent = char(condPathPoint.getElementsByTagName('socket_parent_frame').item(0).getTextContent());
        socketParent = extractAfter(socketParent, '/bodyset/');
        location = char(condPathPoint.getElementsByTagName('location').item(0).getTextContent());
        dof = char(condPathPoint.getElementsByTagName('socket_coordinate').item(0).getTextContent());
        dof = split(dof, '/'); dof = dof{4}; 
        range = char(condPathPoint.getElementsByTagName('range').item(0).getTextContent());

        % stored into MuscleCondPathPointDef class
        p = MuscleConditionalPathPointDef('attachedBody', socketParent, 'location', str2num(location), 'range', str2num(range), 'dof', dof); %#ok<ST2NM>
        muscleArr(i+1) = muscleArr(i+1).add_muscleCondPathPoint(p);
    end

    % 3. Moving path points
    movingPathPoints = pathPointSet.getElementsByTagName('MovingPathPoint');

    % for each path point of the muscle
    for j = 0:movingPathPoints.getLength-1
        movingPathPoint = movingPathPoints.item(j);

        % get socket_parent_frame, location and range of each path point
        socketParent = char(movingPathPoint.getElementsByTagName('socket_parent_frame').item(0).getTextContent());
        socketParent = extractAfter(socketParent, '/bodyset/');
        dof = char(movingPathPoint.getElementsByTagName('socket_x_coordinate').item(0).getTextContent());
        dof = split(dof, '/'); dof = dof{4};
        xlocation = movingPathPoint.getElementsByTagName('x_location').item(0);
        angle = str2num(char(xlocation.getElementsByTagName('x').item(0).getTextContent())); %#ok<ST2NM>
        x = str2num(char(xlocation.getElementsByTagName('y').item(0).getTextContent())); %#ok<ST2NM>
        ppx = spline(angle, x);
        ylocation = movingPathPoint.getElementsByTagName('y_location').item(0);
        angle = str2num(char(ylocation.getElementsByTagName('x').item(0).getTextContent())); %#ok<ST2NM>
        y = str2num(char(ylocation.getElementsByTagName('y').item(0).getTextContent())); %#ok<ST2NM>
        ppy = spline(angle, y);
        zlocation = movingPathPoint.getElementsByTagName('z_location').item(0);
        angle = str2num(char(zlocation.getElementsByTagName('x').item(0).getTextContent())); %#ok<ST2NM>
        z = str2num(char(zlocation.getElementsByTagName('y').item(0).getTextContent())); %#ok<ST2NM>
        ppz = spline(angle, z);

        % stored into MuscleCondPathPointDef class
        p = MuscleMovingPathPointDef('attachedBody', socketParent, 'dof', dof, 'ppx', ppx, 'ppy', ppy, 'ppz', ppz);
        muscleArr(i+1) = muscleArr(i+1).add_muscleMovPathPoint(p);
    end

    %%% --- GET ORDER OF THE PATH POINTS --- %%%
    pathPointOrder = zeros(1, pathPoints.getLength + movingPathPoints.getLength);
    pointer = 1;
    allPathPoints = muscle.getElementsByTagName('objects').item(0).getChildNodes;
    for j = 0 : allPathPoints.getLength-1
        if allPathPoints.item(j).getNodeType == allPathPoints.item(j).ELEMENT_NODE
            nodeName = char(allPathPoints.item(j).getNodeName);
            if strcmp(nodeName, 'PathPoint')
                pathPointOrder(pointer) = 1;
                pointer = pointer + 1;
            elseif strcmp(nodeName, 'ConditionalPathPoint')
                pathPointOrder(pointer) = 2;
                pointer = pointer + 1;
            elseif strcmp(nodeName, 'MovingPathPoint')
                pathPointOrder(pointer) = 3;
                pointer = pointer + 1;
            end
        end
    end
    muscleArr(i+1) = muscleArr(i+1).add_pathPointOrder(pathPointOrder);

end

muscleMap = containers.Map(muscleNames, num2cell(1:muscles.getLength));