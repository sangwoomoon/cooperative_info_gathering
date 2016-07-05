classdef Communication < handle
    properties % ( SetAccess = public, GetAccess = public )
        
        C % communication Matrix (sender : row - receiver : col)
        Z % received package through communication
        
        beta % probability of Empirical Packet model (measured on 03182012)
        
    end % Properties
    
    methods
        function o = Communication( SIMULATION, CLOCK )
             o = Default(o, SIMULATION, CLOCK );
        end
        
        o = get( o, varargin );
    end
    
end



