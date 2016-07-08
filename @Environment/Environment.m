classdef Environment < handle
    properties ( SetAccess = public, GetAccess = public )
        
        xlength     % field size x-axis
        ylength     % field size y-axis
        
        x           % x-position (for detection)
        y           % y-position (for detection)
        delta       % discretized length
        
        bound       % boundary point of field
        
        plot
       
    end % Properties
    
    methods
        function o = Environment ( clock, size, delta )
            o = Default( o, clock, size, delta );
        end
        
        o = get( o, varargin );
    end
    
end



