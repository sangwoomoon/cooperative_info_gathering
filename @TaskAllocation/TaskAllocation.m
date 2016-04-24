classdef TaskAllocation < handle
    properties % ( SetAccess = public, GetAccess = public )
        
        bTasklist % assigned tasks for each agent (determinent set)
        nSearch  % assigned number of targets (sum of tasklist)
        
    end % Properties
    
    methods
        function o = TaskAllocation( SIMULATION, CLOCK )
             o = Default(o, SIMULATION, CLOCK );
        end
        
        o = get( o, varargin );
    end
    
end



