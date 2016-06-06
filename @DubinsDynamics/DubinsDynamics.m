classdef DubinsDynamics < Dynamics
    %DubinsDynamics is a sub-class of Dynamics class
    
    properties
        
    end
    
    methods
        
        function o = DubinsDynamics( )
            o@Dynamics( );
        end
        
        % Differential equation
        % will be used for ODE45
        dx = StateDerivate(obj, t, x, u);
        
        % make process noise
        MakeNoise(obj);
        
    end
    
    
end

