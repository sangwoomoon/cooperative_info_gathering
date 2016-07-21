classdef Agent < handle
    properties ( SetAccess = public, GetAccess = public )
        
        id       % Agent id (integers)
        
        TA       % Task Allocation Class (element of agent)
        COMM     % Communication Class (element of agent)
        
        DYNAMICS % Dynamics Class (element of agent, superclass of Linear/Dubins/LinearBias)
        SENSOR   % Sensor Class (element of agent)
        CONTROL  % Control Class (element of agent)
        
        FDDF     % Factorized DDF Class (element of agent)
       
        ESTIMATOR % Estimator Class (super class of KF, EKF..)
        
        plot % plotting handle (for state and used at the dynamics class)
        
    end % Properties
    
    methods
        function obj = Agent( iAgent, TARGET, SIMULATION, CLOCK , DynamicsOption, SensorOption, EstimatorOption )
             obj = Declare(obj, iAgent, TARGET, SIMULATION, CLOCK , DynamicsOption, SensorOption, EstimatorOption );
        end
        
        % group jacobian : since this function should uses the TARGET,
        % LANDMARK classes. should be modified because AGENT cannot know
        % actual dynamics of TARGET
        jacobian = GatherJacobian(obj, TARGET, CLOCK, option) % option: state / noise
        
    end
    
end



