classdef Sensor < handle
    properties % ( SetAccess = public, GetAccess = public )
        
        spec    % measurement model (e.g. linear, range/bearing ...)
        
        bias    % sensor bias
        meas    % measurement [.id and .y] 
        
        subject % sensing agent? or target?
        
        v       % random measurement noise
        R       % measurement noise covariances
        
        Q       % bias process noise covariances
                
        hist    % History 
        plot    % Plot handle
        
    end % Properties
    
    methods
        function obj = Sensor()
             obj = Declare(obj);
        end
       
        % take measurement
        Measure(obj, s, TARGET, LANDMARK, current_time);
        
        % parameter setting :: measurement noise covariance matrix (R)
        SetParameters(obj, biasQinput, SensedObject, SensorNoiseCov);
        
        % initialize Sensor setting
        InitializeSensor(obj, SensorBias, AgentID);
        
        % Generate Measurement matrix (H)
        % value would be with respect to agent state, target state, and sensor bias
        output = TakeJacobian(obj);
        
        % Generate measurement noise :: should be inside of sub-classes!
        MeasureNoise = MakeNoise(obj);
        
        % plotting function for measurement (in terms of locations)
        Plot(obj, PlottedClass);
        
    end
    
end



