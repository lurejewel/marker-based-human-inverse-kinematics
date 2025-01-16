classdef JointDef

    properties

        translation_parent % 3x1
        Ndof % number of DOF
        axis % 4xNdof: [x y z 0/1] (0 for rotational joint, and 1 for translational joint)
        range % 2xNdof: [min max]
        defaultValue % 1xNdof
        defaultSpeed % 1xNdof

    end

    methods
        
        function obj = JointDef(varargin)
            
            p = inputParser;

            addParameter(p, 'translation_parent', [0 0 0]', @isnumeric);
            addParameter(p, 'Ndof', 1, @isnumeric);
            addParameter(p, 'axis', [0 0 1 0]', @isnumeric);
            addParameter(p, 'range', deg2rad(120)*[-1 1]', @isnumeric);
            addParameter(p, 'defaultValue', 0, @isnumeric);
            addParameter(p, 'defaultSpeed', 0, @isnumeric);

            parse(p, varargin{:});

            obj.axis = p.Results.axis;
            obj.Ndof = p.Results.Ndof;
            obj.defaultValue = p.Results.defaultValue;
            obj.defaultSpeed = p.Results.defaultSpeed;
            obj.range = p.Results.range;
            obj.translation_parent = p.Results.translation_parent;

        end
    end
end