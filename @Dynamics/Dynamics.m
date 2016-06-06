classdef Dynamics < handle
    %DYNAMICS is a member of AGENT or TARGET class
    %   Generic form concerned with dynamics of agents or targets are
    %   handled as properties, and it rules as a placeholder in order to
    %   specify dynamic models.
    
    properties
        
        spec    % dynamic model specification (e.g. Linear / Dubins)
        
        x       % current internal state (without noise)
        x_e     % current external state (with noise)
        
        bKFx    % binary array of state for using KF process.
        
        v      % Random variable wrt movement of agent/target
        Q      % process noise for platform (accel noise)
        
        ODEoption % ODE45 integration option
        
        % history and plotting options
        hist
        plot
        
    end
    
    methods 
        
        % constructor method
        % or specified version of constructor
        function obj = Dynamics()
            obj = Default(obj); % use construct (function name)
        end
            
        % Propagate State
        % update internal state and history
        % Time update with respect to dynamics (overloading scheme)
        TimeUpdate(obj, u, CLOCK);
        
        % State Update
        % update external state and history
        StateUpdate(obj, u, CLOCK);
        
        % Predict State
        % State Prediction method (N-step ahead prediction with zero noise)
        PredictState(obj, CLOCK, u, N);
        
        % Take Jacobian matrix method (output is not object itself, so
        % output parameters should be remained)
        output = TakeJacobian(obj, equation, value);
        
        % Set parameters by users
        SetParameters(obj, bKFx, Q, RelTol, AbsTol);
        
        % Initialize states by users
        InitializeState(obj, x);
        
    end
    
    
end

