classdef Environment < handle
    properties ( SetAccess = public, GetAccess = public )
    
        LANDMARK % set as stationary target
       
    end % Properties
    
    methods
        function o = Environment ( clock )
            o = Declare( o, clock );
        end
        
        o = get( o, varargin );
    end
    
end



