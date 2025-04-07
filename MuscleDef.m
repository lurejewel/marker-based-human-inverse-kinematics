classdef MuscleDef

    properties

        % basic information about the muscle
        name % char
        mvc % max isometric force / max voluntary contraction force
        optFiberLen % optimal fiber length
        tendonSlackLen % tendon slack length
        penAngleAtOpt % pennation angle (angle between tendon and fibers) at optimal fiber length (in rad)
        FmaxTendonStrain % tendon strain at mvc
        FmaxMuscleStrain % passive muscle strain at mvc
        KshapeActive % shape factor for Gaussian active muscle force-length relationship
        KshapePassive % exponential shape factor for passive force-length relationship
        Af % forfce-velocity shape factor
        Flen % max normalized lengthening force
        actTime % activation time constant (in sec)
        deactTime % deactivation time constant (in sec)

        % general path points
        nPathPoints % number of muscle path points
        pathPoints % nx1 MusclePathPointDef class

        % conditional path points
        nCondPathPoints % number of muscle conditional path points
        condPathPoints % mx1 MuscleCondPathPointDef class

        % moving path points
        nMovingPathPoints % number of muscle moving path points
        movingPathPoints % lx1 MuslceMovingPathPointDef class
        
        % order of path points

        
        pathPointOrder % (n+m+l)x1 array. 1 for general path point, 2 for conditional path point, and 3 for moving path point

        % calculated information
        muscleLength % muscle length (fiber + tendon)

    end

    methods
        
        function obj = MuscleDef(varargin)

            p = inputParser;

            addParameter(p, 'name', 'default', @ischar);
            addParameter(p, 'mvc', 0, @isnumeric);
            addParameter(p, 'optFiberLen', 0, @isnumeric);
            addParameter(p, 'tendonSlackLen', 0, @isnumeric);
            addParameter(p, 'penAngleAtOpt', 0, @isnumeric);
            addParameter(p, 'FmaxTendonStrain', 0, @isnumeric);
            addParameter(p, 'FmaxMuscleStrain', 0, @isnumeric);
            addParameter(p, 'KshapeActive', 0, @isnumeric);
            addParameter(p, 'KshapePassive', 0, @isnumeric);
            addParameter(p, 'Af', 0, @isnumeric);
            addParameter(p, 'Flen', 0, @isnumeric);
            addParameter(p, 'actTime', 0, @isnumeric);
            addParameter(p, 'deactTime', 0, @isnumeric);
            addParameter(p, 'nPathPoints', 0, @isnumeric);
            addParameter(p, 'pathPoints', MusclePathPointDef(), @isnumeric);
            addParameter(p, 'nCondPathPoints', 0, @isnumeric);
            addParameter(p, 'condPathPoints', MuscleConditionalPathPointDef(), @isnumeric);
            addParameter(p, 'nMovingPathPoints', 0, @isnumeric);
            addParameter(p, 'movingPathPoints', MuscleMovingPathPointDef(), @isnumeric);
            addParameter(p, 'pathPointOrder', 0, @isnumeric);
            addParameter(p, 'muscleLength', 0, @isnumeric);

            parse(p, varargin{:});

            obj.name = p.Results.name;
            obj.mvc = p.Results.mvc;
            obj.optFiberLen = p.Results.optFiberLen;
            obj.tendonSlackLen = p.Results.tendonSlackLen;
            obj.penAngleAtOpt = p.Results.penAngleAtOpt;
            obj.FmaxTendonStrain = p.Results.FmaxTendonStrain;
            obj.FmaxMuscleStrain = p.Results.FmaxMuscleStrain;
            obj.KshapeActive = p.Results.KshapeActive;
            obj.KshapePassive = p.Results.KshapePassive;
            obj.Af = p.Results.Af;
            obj.Flen = p.Results.Flen;
            obj.actTime = p.Results.actTime;
            obj.deactTime = p.Results.deactTime;
            obj.nPathPoints = p.Results.nPathPoints;
            obj.pathPoints = p.Results.pathPoints;
            obj.nCondPathPoints = p.Results.nCondPathPoints;
            obj.condPathPoints = p.Results.condPathPoints;
            obj.nMovingPathPoints = p.Results.nMovingPathPoints;
            obj.movingPathPoints = p.Results.movingPathPoints;
            obj.pathPointOrder = p.Results.pathPointOrder;
            obj.muscleLength = p.Results.muscleLength;

        end

        function obj = add_musclePathPoint(obj, musclePathPoint)

            obj.nPathPoints = obj.nPathPoints + 1;
            obj.pathPoints(obj.nPathPoints) = musclePathPoint;

        end

        function obj = add_muscleCondPathPoint(obj, muscleCondPathPoint)

            obj.nCondPathPoints = obj.nCondPathPoints + 1;
            obj.condPathPoints(obj.nCondPathPoints) = muscleCondPathPoint;

        end

        function obj = add_muscleMovPathPoint(obj, muscleMovingPathPoint)

            obj.nMovingPathPoints = obj.nMovingPathPoints + 1;
            obj.movingPathPoints(obj.nMovingPathPoints) = muscleMovingPathPoint;

        end

        function obj = add_pathPointOrder(obj, pathPointOrder)

            obj.pathPointOrder = pathPointOrder;

        end

    end
end