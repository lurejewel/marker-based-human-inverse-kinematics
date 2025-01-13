classdef BodyDef

    properties

        T % 4x4 Transformation matrix indicating position and orientation
        mass
        inertia % 6x1 [Ixx Iyy Izz Ixy Ixz Iyz]'
        COM % 3x1 center of mass w.r.t. the origin frame of the body

    end

    methods
        
        function obj = BodyDef(varargin)
            
            p = inputParser;

            addParameter(p, 'T', eye(4), @isnumeric);
            addParameter(p, 'mass', 0, @isnumeric);
            addParameter(p, 'inertia', zeros(6,1), @isnumeric);
            addParameter(p, 'COM', zeros(3,1), @isnumeric);

            parse(p, varargin{:});

            obj.COM = p.Results.COM;
            obj.inertia = p.Results.inertia;
            obj.T = p.Results.T;
            obj.mass = p.Results.mass;

        end
    end
end