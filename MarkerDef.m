classdef MarkerDef
    
    properties
        
        attachedBody % name of the body frame it's attached to
        deviation % 3x1 [x y z]' relative to attached Body
        location % 3x1 [x y z]' in world coordinate system

    end
    
    methods
        
        function obj = MarkerDef(varargin)

            p = inputParser;
            
            addParameter(p, 'attachedBody', 'none', @ischar);
            addParameter(p, 'deviation', zeros(3,1), @isnumeric);
            addParameter(p, 'location', zeros(3,1), @isnumeric);

            parse(p, varargin{:});

            obj.attachedBody = p.Results.attachedBody;
            obj.deviation = p.Results.deviation;
            obj.location = p.Results.location;

        end
    end
end

