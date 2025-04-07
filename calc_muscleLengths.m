function muscles = calc_muscleLengths(bodies, muscles, q, dofMap)
%% read input data

pelvis = bodies.pelvis;
femur_r = bodies.femur_r;
femur_l = bodies.femur_l;
tibia_r = bodies.tibia_r;
tibia_l = bodies.tibia_l;
foot_r = bodies.foot_r;
foot_l = bodies.foot_l;
lumbar = bodies.lumbar;
thorax = bodies.thorax;
head = bodies.head;
arm_r = bodies.arm_r; %#ok<*NASGU>
arm_l = bodies.arm_l;
forearm_r = bodies.forearm_r;
forearm_l = bodies.forearm_l;
hand_r = bodies.hand_r;
hand_l = bodies.hand_l;

%% calculate all muscle lengths

for i = 1 : length(muscles) % for every muscle

    allPathPointLocations = zeros(4, muscles(i).nPathPoints + muscles(i).nCondPathPoints);
    
    % locations of (general) path points
    pathPointLocations = zeros(4, muscles(i).nPathPoints);
    for j = 1 : muscles(i).nPathPoints % for every (general) path point
        if isequal(muscles(i).pathPoints(j).attachedBody, 'calcn_r')
            muscles(i).pathPoints(j).attachedBody = 'foot_r';
            % muscles(i).pathPoints(j).location = muscles(i).pathPoints(j).location + [-0.04877 -0.04195 0.00792];
        elseif isequal(muscles(i).pathPoints(j).attachedBody, 'calcn_l')
            muscles(i).pathPoints(j).attachedBody = 'foot_l';
            % muscles(i).pathPoints(j).location = muscles(i).pathPoints(j).location + [-0.04877 -0.04195 -0.00792];
        % elseif isequal(muscles(i).pathPoints(j).attachedBody, 'toes_r')
        %     muscles(i).pathPoints(j).attachedBody = 'foot_r';
        %     muscles(i).pathPoints(j).location = muscles(i).pathPoints(j).location + [0.1788 -0.002 0.00108] + [-0.04877 -0.04195 0.00792];
        % elseif isequal(muscles(i).pathPoints(j).attachedBody, 'toes_l')
        %     muscles(i).pathPoints(j).attachedBody = 'foot_l';
        %     muscles(i).pathPoints(j).location = muscles(i).pathPoints(j).location + [0.1788 -0.002 -0.00108] + [-0.04877 -0.04195 -0.00792];
        end
        pathPointLocations(:,j) = eval(muscles(i).pathPoints(j).attachedBody).T * [muscles(i).pathPoints(j).location'; 1];
    end

    % locations of conditional path points
    condPathPointLocations = zeros(4, muscles(i).nCondPathPoints);
    for j = 1 : muscles(i).nCondPathPoints % for every conditional path point

        if isequal(muscles(i).condPathPoints(j).attachedBody, 'calcn_r')
            muscles(i).condPathPoints(j).attachedBody = 'foot_r';
            % muscles(i).condPathPoints(j).location = muscles(i).condPathPoints(j).location + [-0.04877 -0.04195 0.00792];
        elseif isequal(muscles(i).condPathPoints(j).attachedBody, 'calcn_l')
            muscles(i).condPathPoints(j).attachedBody = 'foot_l';
            % muscles(i).condPathPoints(j).location = muscles(i).condPathPoints(j).location + [-0.04877 -0.04195 -0.00792];
        % elseif isequal(muscles(i).condPathPoints(j).attachedBody, 'toes_r')
        %     muscles(i).condPathPoints(j).attachedBody = 'foot_r';
        %     muscles(i).condPathPoints(j).location = muscles(i).condPathPoints(j).location + [0.1788 -0.002 0.00108] + [-0.04877 -0.04195 0.00792];
        % elseif isequal(muscles(i).condPathPoints(j).attachedBody, 'toes_l')
        %     muscles(i).condPathPoints(j).attachedBody = 'foot_l';
        %     muscles(i).condPathPoints(j).location = muscles(i).condPathPoints(j).location + [0.1788 -0.002 -0.00108] + [-0.04877 -0.04195 -0.00792];
        end
        condPathPointLocations(:,j) = eval(muscles(i).condPathPoints(j).attachedBody).T * [muscles(i).condPathPoints(j).location'; 1];

    end

    % locations of moving path points
    movingPathPointLocations = zeros(4, muscles(i).nMovingPathPoints);
    for j = 1 : muscles(i).nMovingPathPoints % for every moving path point

        if isequal(muscles(i).movingPathPoints(j).attachedBody, 'calcn_r')
            muscles(i).movingPathPoints(j).attachedBody = 'foot_r';
            % muscles(i).movingPathPoints(j).location = muscles(i).movingPathPoints(j).location + [-0.04877 -0.04195 0.00792];
        elseif isequal(muscles(i).movingPathPoints(j).attachedBody, 'calcn_l')
            muscles(i).movingPathPoints(j).attachedBody = 'foot_l';
            % muscles(i).movingPathPoints(j).location = muscles(i).movingPathPoints(j).location + [-0.04877 -0.04195 -0.00792];
        % elseif isequal(muscles(i).movingPathPoints(j).attachedBody, 'toes_r')
        %     muscles(i).movingPathPoints(j).attachedBody = 'foot_r';
        %     muscles(i).movingPathPoints(j).location = muscles(i).movingPathPoints(j).location + [0.1788 -0.002 0.00108] + [-0.04877 -0.04195 0.00792];
        % elseif isequal(muscles(i).movingPathPoints(j).attachedBody, 'toes_l')
        %     muscles(i).movingPathPoints(j).attachedBody = 'foot_l';
        %     muscles(i).movingPathPoints(j).location = muscles(i).movingPathPoints(j).location + [0.1788 -0.002 -0.00108] + [-0.04877 -0.04195 -0.00792];
        end
        dof = muscles(i).movingPathPoints(j).dof;
        ppx = muscles(i).movingPathPoints(j).ppx;
        ppy = muscles(i).movingPathPoints(j).ppy;
        ppz = muscles(i).movingPathPoints(j).ppz;
        location = [ppval(ppx, q(dofMap(dof))), ppval(ppy, q(dofMap(dof))), ppval(ppz, q(dofMap(dof)))];
        movingPathPointLocations(:,j) = eval(muscles(i).movingPathPoints(j).attachedBody).T * [location'; 1];

    end

    % place the points in order
    pathPointPointer = 1;
    condPathPointPointer = 1;
    movPathPointPointer = 1;
    % movingPathPointPointer = 1;
    for j = 1 : length(muscles(i).pathPointOrder)
        switch muscles(i).pathPointOrder(j)
            case 1 % (general) path point
                allPathPointLocations(:,j) = pathPointLocations(:,pathPointPointer);
                pathPointPointer = pathPointPointer + 1;
            case 2 % conditional path point
                dof = muscles(i).condPathPoints(condPathPointPointer).dof;
                min = muscles(i).condPathPoints(condPathPointPointer).range(1);
                max = muscles(i).condPathPoints(condPathPointPointer).range(2);
                if q(dofMap(dof)) > min && q(dofMap(dof)) < max % check if the conditional path point is activated (the determining dof is within range)
                    allPathPointLocations(:,j) = condPathPointLocations(:,condPathPointPointer);
                else
                    allPathPointLocations(:,j) = allPathPointLocations(:,j-1);
                end
                condPathPointPointer = condPathPointPointer + 1;
            case 3 % moving path point
                allPathPointLocations(:,j) = movingPathPointLocations(:,movPathPointPointer);
                movPathPointPointer = movPathPointPointer + 1;
            otherwise
                error('Unexpected muscle path point order. check "muscles(i).pathPointOrder" to debug.');
        end
    end
    % calculate length of the poly line (defined by all muscle path points)
    muscles(i).muscleLength = calc_PolylineLength(allPathPointLocations(1:3,:));

end

end