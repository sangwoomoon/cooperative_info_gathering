classdef Communication < handle
    properties % ( SetAccess = public, GetAccess = public )
        
        % C % communication Matrix (sender : row - receiver : col)
        Z % received package through communication
        
    end % Properties
    
    methods
        function o = Communication( SIMULATION, CLOCK )
             o = Declare(o, SIMULATION, CLOCK );
        end
        
        o = get( o, varargin );
    end
    
end



