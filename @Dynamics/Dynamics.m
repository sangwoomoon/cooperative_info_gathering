classdef Dynamics < handle
    %DYNAMICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        % symbolic form
        s_sym   % symbolic form of states of agent
        v_sym   % symbolic form of input noise
        
        Eqn     % symbolic function form of equation of motion
        
        F      % State transition matrix
        Gamma    % Process noise input matrix
        
        Gu      % Control input matrix
        
        
        % numeric form
        s       % current state [b_e, b_n, e,edot,n,ndot]
        bKFs    % binary array of state for using KF process.
        
        v      % Random variable wrt movement of agent/target
        Q      % process noise for platform (accel noise)
        
    end
    
    methods
        function o = Dynamics( SIMULATION, CONTROL, CLOCK )
            o = Default(o, SIMULATION, CONTROL, CLOCK );
        end
        
        o = get( o, varargin );
    end
    
end

