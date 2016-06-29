classdef TaskAllocation < handle
    properties % ( SetAccess = public, GetAccess = public )
        
        TrackID  % track (measured) taraget ID from TA
        
        hist
        plot
        
    end % Properties
    
    methods
        function o = TaskAllocation( TARGET, CLOCK )
             o = Default(o, TARGET, CLOCK );
        end
        
        o = get( o, varargin );
    end
    
end



