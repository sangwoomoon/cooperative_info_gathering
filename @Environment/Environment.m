classdef Environment < handle
    properties ( SetAccess = public, GetAccess = public )
        
        xlength     % field size x-axis
        ylength     % field size y-axis
        
        bound       % boundary point of field
       
    end % Properties
    
    methods
        function o = Environment ( clock, size )
            o = Default( o, clock, size );
        end
        
        o = get( o, varargin );
    end
    
end



