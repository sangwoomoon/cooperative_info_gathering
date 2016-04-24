classdef Environment < handle
    properties ( SetAccess = public, GetAccess = public )
        
        xlength     % field size x-axis
        ylength     % field size y-axis
        
        kr          % ratio for centering agents and targets (between 0 to 1)
        
        bound       % boundary point of field
       
    end % Properties
    
    methods
        function o = Environment ( clock )
            o = Default( o, clock );
        end
        
        o = get( o, varargin );
    end
    
end



