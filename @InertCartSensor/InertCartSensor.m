classdef InertCartSensor < Sensor
    %INERTCARTSENSOR is a sub-class of Sensor class
    %   measurement : [abs_e, abs_n]'
    
    properties
               
    end
    
    methods
        
        function obj = InertCartSensor()
            obj@Sensor();
        end
        
        % Take measurement for other agents/targets
        Measure(obj, s, x, landmark, current_time);
        
        % make measurement noise
        MakeNoise(obj, option);
        
        % Initialize measurement data and its history
        InitializeData(obj);
        
        % Plot History of states (environment class is for plotting measurement for landmarks 
        Plot(obj, ENVIRONMENT);
        
        % parameter setting :: measurement noise covariance matrix (R),
        % sensed object.
        % dependent on the subclasses of Sensor class
        SetParameters(obj, SensedObject, bTrackTarget, SensorNoiseCov);
        
        % make jacobian for SINGLE target (with respect to targetID)
        Jacobian = TakeTargetJacobian(obj, measTargetID);
        
        % make jacobian for bias
        Jacobian = TakeBiasJacobian(obj, option, measurement); % measurement is only for taking H
        
    end
    
end

