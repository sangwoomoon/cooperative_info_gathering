classdef DubinsDynamics < Dynamics
    %DubinsDynamics is a sub-class of Dynamics class
    
    properties
        
    end
    
    methods
        
        function o = DubinsDynamics(CONTROL, CLOCK, id, option )
            o@Dynamics(CONTROL, CLOCK, id, option );
        end
        
    end
    
end

