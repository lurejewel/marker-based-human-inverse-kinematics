classdef MuscleMovingPathPointDef

    properties

        attachedBody % string, name of a bone (body) where the path point is attached to
        dof % string, the dof (coordinate) that determines the value of the point
        ppx % interpolation function: x = ppx(dof)
        ppy % interpolation function: y = ppy(dof)
        ppz % interpolation function: z = ppz(dof)

    end

    methods
        
        function obj = MuscleMovingPathPointDef(varargin)
            
            p = inputParser;

            addParameter(p, 'attachedBody','ground', @ischar);
            addParameter(p, 'dof', 'none', @ischar);
            addParameter(p, 'ppx', []);
            addParameter(p, 'ppy', []);
            addParameter(p, 'ppz', []);

            parse(p, varargin{:});

            obj.attachedBody = p.Results.attachedBody;
            obj.dof = p.Results.dof;
            obj.ppx = p.Results.ppx;
            obj.ppy = p.Results.ppy;
            obj.ppz = p.Results.ppz;

        end
    end
end