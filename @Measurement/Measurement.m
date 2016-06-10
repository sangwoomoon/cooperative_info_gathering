classdef Measurement < handle
    properties % ( SetAccess = public, GetAccess = public )
        
        spec    % measurement model (e.g. linear, range/bearing ...)
        mode    % is it measured at the current time? (on/off)
        
        subject % who is the measured one? (e.g. target/agent)
        id      % what is the number of agent/target to be measured?
        
        y       % measurement [e_rel,n_rel,e_abs,n_abs] 
        
        w       % random measurement noise
        R       % measurement noise variances
                
        hist    % History 
        plot    % Plot handle
        
    end % Properties
    
    methods
        function obj = Measurement()
             obj = Declare(obj);
        end
       
        % parameter setting :: measurement noise covariance matrix (R)
        SetParameters(obj, R, mode);
        
        % initialize Mesurement setting
        InitializeMeausre(obj);
        
        % Generate Measurement matrix (H)
        % value would be with respect to agent state, target state, and sensor bias
        output = TakeJacobian(obj, equation, value);
        
        % Generate measurement noise :: should be inside of sub-classes!
        MakeNoise(obj);
        
        % plotting function for measurement (in terms of locations)
        Plot(obj, PlottedClass);
        
        % convert states in order to take measurement (standardize
        % coordinate to linear-bias coordinate (zero value if no bias in
        % previous coordiate)
        ConverState(obj, beforeSpec);
        
    end
    
end



