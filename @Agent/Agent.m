classdef Agent < handle
    properties ( SetAccess = public, GetAccess = public )
        
        id      % Agent id (integers)
        
        TA      % Task Allocation Class (element of agent)
        COMM    % Communication Class (element of agent)
        
        DYNAMICS% Dynamics Class (element of agent, superclass of Linear/Dubins/LinearBias)
        MEASURE % Measurement Class (element of agent)
        CONTROL % Control Class (element of agent)
        
        FDDF    % Factorized DDF Class (element of agent)
       
        ESTIMATOR % Estimator Class (super class of KF, EKF..)
        
    end % Properties
    
    methods
        function o = Agent( TARGET, ENVIRONMENT, SIMULATION, CLOCK ,iAgent, option )
             o = Default(o, TARGET, ENVIRONMENT, SIMULATION, CLOCK ,iAgent, option );
        end
        
    end
    
end



