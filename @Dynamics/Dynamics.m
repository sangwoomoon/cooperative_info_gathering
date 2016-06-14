classdef Dynamics < handle
    %DYNAMICS is a member of AGENT or TARGET class
    %   Generic form concerned with dynamics of agents or targets are
    %   handled as properties, and it rules as a placeholder in order to
    %   specify dynamic models.
    
    properties
        
        spec    % dynamic model specification (e.g. Linear / Dubins)
        
        x       % current state
        x_pred  % predicted state
        
        bKFx    % binary array of state for using KF process.
        
        v      % Random variable wrt movement of agent/target
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
        TimeUpdate(obj, u, CLOCK);
        
        % Predict State
        % State Prediction method (N-step ahead prediction with zero noise)
        % use the StateUpdate function to update state, and then store data
        PredictState(obj, x, u, w, CLOCK);
        
        % State Update only (this function is used to TimeUpdate /
        % PredictState)
        [x,F,Gamma,Gu] = StateUpdate(obj, x, u, w, CLOCK); 
        
        % differential equation that is called by StateUpdatefunction
        % set as a template and it must be defined into the sub-class
        dx = StateDerivate(obj, t, x, u, w);
        
        % make process noise
        % set as a template and it must be defined into the sub-class        
        w = MakeNoise(obj);
        
%      % JACOBIAN PART - now included in StateUpdate in one unique process
%         
%         % take jacobian matrix (STM, Gamma, Gu)
%         jacobian = JacobianUpdate(obj, x, u, w, dt, option);
%         
%         djacobian = TakeJacobian( obj, jacobian, x, u, w, option );
        
     % PARAMETER PART
        
        % Set parameters by users (so far we don't need to overload)
        SetParameters(obj, bKFx, process_noise, RelTol, AbsTol);
        
        % Initialize states by users
        InitializeState(obj, x_initial);
        
     % PLOT PART
        
        % Plot History of states 
        % it is specified with respect to (plotting options are on the AGENT/TARGET
        % class!)
        % set as a template and it must be defined into the sub-class
        Plot(obj, PlottedClass);
        
    end
    
    
end

