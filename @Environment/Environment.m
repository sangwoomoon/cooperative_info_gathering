classdef Environment < handle
    properties ( SetAccess = public, GetAccess = public )
    
       
    end % Properties
    
    methods
        function o = Environment ( clock )
            o = Default( o, clock );
        end
        
        o = get( o, varargin );
    end
    
end



