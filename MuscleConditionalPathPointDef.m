classdef MuscleConditionalPathPointDef

    properties

        attachedBody % string, name of a bone (body) where the path point is attached to
        location % [x y z], deviation from the bone, expressed in the bone's axis
        dof % string, the dof (coordinate) that determines the activation of the point
        range % [min max] where the conditional path point is activated

    end

    methods
        
        function obj = MuscleConditionalPathPointDef(varargin)
            
            p = inputParser;

            addParameter(p, 'attachedBody','ground', @ischar);
            addParameter(p, 'location', [0 0 0], @isnumeric);
            addParameter(p, 'range', [0 0], @isnumeric);
            addParameter(p, 'dof', 'none', @ischar);

            parse(p, varargin{:});

            obj.attachedBody = p.Results.attachedBody;
            obj.location = p.Results.location;
            obj.range = p.Results.range;
            obj.dof = p.Results.dof;

        end
    end
end