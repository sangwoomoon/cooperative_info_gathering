classdef Sensor < handle
    properties % ( SetAccess = public, GetAccess = public )
        
        spec    % measurement model (e.g. linear, range/bearing ...)
        nState  % sensor measurement state (for estimation)
        
        meas    % measurement [.id and .y] 
        
        subject % sensing agent? or target?
        
        v       % random measurement noise
        R       % measurement noise covariances
                        
        hist    % History 
        plot    % Plot handle
        
    end % Properties
    
    methods
        function obj = Sensor()
             obj = Declare(obj);
        end
       
        % take measurement
        Measure(obj, s, TARGET, LANDMARK, current_time);
        
        % parameter setting :: measurement noise covariance matrix (R),
        % sensed object.
        % dependent on the subclasses of Sensor class
        SetParameters(obj, SensedObject, SensorNoiseCov);
        
        % initialize Sensor setting
        Initialize(obj, SensorBias, AgentID);
        
        % Generate Measurement matrix (H)
        % value would be with respect to agent state, target state, and sensor bias
        output = TakeJacobian(obj, option);
        
        % Generate measurement noise :: should be inside of sub-classes!
        MeasureNoise = MakeNoise(obj);
        
        % plotting function for measurement (in terms of locations)
        Plot(obj, PlottedClass);
        
        % gathers measurements for all targets to use in Estimator class
        Y = GatherMeasurements( obj );
        
        % gathers measurement noise covariance matrix to use in Estimator
        % class
        R = GatherMeasNoiseCovMatrix(obj);

                
    end
    
end



