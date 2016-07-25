classdef StaticDynamics < Dynamics
    %LinearDynamics is a sub-class of Dynamics class
    
    properties
        
    end
    
    methods

        function obj = StaticDynamics()
            obj@Dynamics();
        end
        
        % take jacobian matrix (STM, Gamma, Gu)
        jacobian = TakeJacobian(obj, u, dt, option);
        
        % state update : should be the same position and state because it
        % is static
        stateNext = StateUpdate(obj, x, u, w, CLOCK);
        
        % take current position
        currentPosition = GetPosition(obj);
        
        % Plot History of states 
        % it is specified with respect to (plotting options are on the AGENT/TARGET
        % class!)
        Plot(obj, PlottedClass);
        
    end
    
end

