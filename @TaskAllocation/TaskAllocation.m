classdef TaskAllocation < handle
    properties % ( SetAccess = public, GetAccess = public )
        
    end % Properties
    
    methods
        function o = TaskAllocation( TARGET, CLOCK )
             o = Default(o, TARGET, CLOCK );
        end
        
        o = get( o, varargin );
    end
    
end



