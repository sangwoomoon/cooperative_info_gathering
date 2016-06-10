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
        dx = StateDerivate(obj, t, x, u);
        
        % take jacobian matrix (STM, Gamma, Gu)
        jacobian = TakeJacobian(obj, u, dt, option);
        
        % make process noise
        v = MakeNoise(obj); 
        
        % Plot History of states 
        % it is specified with respect to (plotting options are on the AGENT/TARGET
        % class!)
        Plot(obj, PlottedClass);
    end
    
end

