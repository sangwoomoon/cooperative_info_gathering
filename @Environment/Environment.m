classdef Environment < handle
    properties ( SetAccess = public, GetAccess = public )
        
        xlength     % field size x-axis
        ylength     % field size y-axis
        
        bound       % boundary point of field
       
    end % Properties
    
    methods
        function o = Environment ( clock )
            o = Default( o, clock );
        end
        
        o = get( o, varargin );
    end
    
end



