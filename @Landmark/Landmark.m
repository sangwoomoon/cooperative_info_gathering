classdef Landmark < handle
    properties ( SetAccess = public, GetAccess = public )
        
        id      % landmark nametag (integers)
        
        DYNAMICS % Dynamics class (static dynamics)
        
        plot % plotting options (for state)
        
    end % Properties
    
    methods
        function obj = Landmark( landmarkID, option )
            obj = Declare( obj, landmarkID, option );
        end
        
    end
    
end



