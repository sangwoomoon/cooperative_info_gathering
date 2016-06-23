classdef Dynamics < handle
    %DYNAMICS is a member of AGENT or TARGET class
    %   Generic form concerned with dynamics of agents or targets are
    %   handled as properties, and it rules as a placeholder in order to
    %   specify dynamic models.
    
    properties
        
        spec    % dynamic model specification (e.g. Linear / Dubins)
        
        x       % current state
        
        bKFx    % binary array of state for using KF process.
        
        w      % Random variable wrt movement of agent/target
        Q      % process noise for platform (accel noise)
        
        ODEoption % ODE45 integration option
        
        % history and plotting options 
        hist
        
    end
    
    methods
        
     % CONSTRUCTOR PART
        
        % constructor method
        % or specified version of constructor
        function obj = Dynamics()
            obj = Declare(obj); % use construct (function name)
        end
            
     % DYNAMICS UPDATE PART
        
        % Propagate State
        % update state and history
        % Time update with respect to dynamics
        % use the StateUpdate function to update state, and then store data
        PropagateState(obj, CurrentState, Input, CLOCK);
        
        % Predict State
        % State Prediction method (N-step ahead prediction with zero noise)
        % use the StateUpdate function to update state, and then store data
        PredictState(obj, CurrentState, InputArray, ProcessNoiseArray, CLOCK);
        
        % State Update only (this function is used into TimeUpdate /
        % PredictState)
        [PredictedState,StateJacobian,NoiseJacobian,InputJacobian] = ...
            StateUpdate(obj, CurrentState, Input, ProcessNoise, CLOCK); 
        
        % differential equation that is called by StateUpdatefunction
        % set as a template and it must be defined into the sub-class
        dx = StateDerivative(obj, Time, CurrentState, Input, ProcessNoise);
        
        % make Gaussian process noise
        % set as a template and it must be defined into the sub-class        
        ProcessNoise = MakeNoise(obj, NoiseOption);
        
        
     % PARAMETER PART
        
        % Set parameters by users (so far we don't need to overload)
        SetParameters(obj, FlagForKF, ProcessNoiseCov, RelativeTolerance, AbsoluteTolerance);
        
        % Initialize states by users
        InitializeState(obj, xInitial);
        
     % PLOT PART
        
        % Plot History of states 
        % it is specified with respect to (plotting options are on the AGENT/TARGET
        % class!)
        % set as a template and it must be defined into the sub-class
        Plot(obj, PlottedClass);
        
    end
    
    
end

