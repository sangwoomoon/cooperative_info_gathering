classdef RelCartBiasSensor < Sensor
    %LINEARMEASURE is a sub-class of Meausrement class
    %   measurement : [abs_e, abs_n, rel_e, rel_n]'
    
    properties
        
        bias    % sensor bias
        Q       % sensor bias noise cov matrix
        
    end
    
    methods
        
        function obj = RelCartBiasSensor()
            obj@Sensor();
        end
        
        % Take measurement for other agents/targets
        Measure(obj, s, TARGET, LANDMARK, current_time);
        
        % make measurement noise
        MakeNoise(obj, option);
        
        % parameter setting :: measurement noise covariance matrix (R),
        % sensed object.
        % dependent on the subclasses of Sensor class
        SetParameters(obj, SensedObject, bias, biasQinput, SensorNoiseCov);
        
        % Initialize measurement data and its history
        InitializeData(obj);
        
        % Plot History of states (environment class is for plotting measurement for landmarks 
        Plot(obj, ENVIRONMENT);
        
        % make jacobian for SINGLE target (with respect to targetID)
        Jacobian = TakeTargetJacobian(obj, measTargetID);
        
        % make jacobian for bias
        Jacobian = TakeBiasJacobian(obj, option);
    end
    
end

