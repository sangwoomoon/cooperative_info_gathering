classdef Measurement < handle
    properties % ( SetAccess = public, GetAccess = public )
        
        spec    % measurement model (e.g. linear, range/bearing ...)
        mode    % is it measured at the current time? (on/off)
        
        subject % who is the measured one? (e.g. target/agent)
        id      % what is the number of agent/target to be measured?
        
        y       % measurement [e_rel,n_rel,e_abs,n_abs] 
        
        v       % random measurement noise
        R       % measurement noise variances
                
        hist    % History 
        plot    % Plot handle
        
    end % Properties
    
    methods
        function obj = Measurement()
             obj = Declare(obj);
        end
       
        % parameter setting :: measurement noise covariance matrix (R)
        SetParameters(obj, MeasureNoiseCov, ActionMode);
        
        % initialize Mesurement setting
        InitializeMeausre(obj);
        
        % Generate Measurement matrix (H)
        % value would be with respect to agent state, target state, and sensor bias
        output = TakeJacobian(obj);
        
        % Generate measurement noise :: should be inside of sub-classes!
        MeasureNoise = MakeNoise(obj);
        
        % plotting function for measurement (in terms of locations)
        Plot(obj, PlottedClass);
        
    end
    
end



