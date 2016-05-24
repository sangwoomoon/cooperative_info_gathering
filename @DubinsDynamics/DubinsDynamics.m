classdef DubinsDynamics < Dynamics
    %DubinsDynamics is a sub-class of Dynamics class
    
    properties
        
    end
    
    methods
        function o = DubinsDynamics( SIMULATION, CONTROL, CLOCK )
            o@Dynamics(SIMULATION, CONTROL, CLOCK );
        end
        
        o = get( o, varargin );
    end
    
end

