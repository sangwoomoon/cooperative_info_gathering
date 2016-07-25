classdef Environment < handle
    properties ( SetAccess = public, GetAccess = public )
    
        LANDMARK % set as stationary target
       
    end % Properties
    
    methods
        function obj = Environment ( )
            obj = Declare( obj );
        end
        
    end
    
end



