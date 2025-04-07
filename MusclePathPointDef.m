classdef MusclePathPointDef

    properties

        attachedBody % string, name of a bone (body) where the path point is attached to
        location % [x y z], deviation from the bone, expressed in the bone's axis

    end

    methods
        
        function obj = MusclePathPointDef(varargin)
            
            p = inputParser;

            addParameter(p, 'attachedBody','ground', @ischar);
            addParameter(p, 'location', [0 0 0], @isnumeric);

            parse(p, varargin{:});

            obj.attachedBody = p.Results.attachedBody;
            obj.location = p.Results.location;

        end
    end
end