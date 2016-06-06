classdef LinearBiasDynamics < Dynamics
    %LinearBiasDynamics is a sub-class of Dynamics class
    
    properties
        
    end
    
    methods
        function o = LinearBiasDynamics()
            o@Dynamics();
        end
        
        % Differential equation
        % will be used for ODE45
        dx = StateDerivate(t, x, u, v);
        
        % make process noise
        MakeNoise(obj);
    end
    
end

