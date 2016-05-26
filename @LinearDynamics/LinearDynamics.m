classdef LinearDynamics < Dynamics
    %LinearDynamics is a sub-class of Dynamics class
    
    properties
        
    end
    
    methods
        function o = LinearDynamics( CONTROL, CLOCK, id, option )
            o@Dynamics(CONTROL, CLOCK, id, option );
        end
        
        o = get( o, varargin );
    end
    
end

