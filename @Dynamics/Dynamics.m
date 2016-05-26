classdef Dynamics < handle
    %DYNAMICS is a member of AGENT or TARGET class
    %   Generic form concerned with dynamics of agents or targets are
    %   handled as properties, and it rules as a placeholder in order to
    %   specify dynamic models.
    
    properties
        
        spec    % dynamic model specification (e.g. Linear / Dubins)
        
        % symbolic form
        s_sym   % symbolic form of states of agent
        v_sym   % symbolic form of input noise
        
        Eqn     % symbolic function form of equation of motion
        
        F       % State transition matrix
        Gamma   % Process noise input matrix
        Gu      % Control input matrix
        
        
        % numeric form
        s       % current state [b_e, b_n, e,edot,n,ndot]
        bKFs    % binary array of state for using KF process.
        
        v      % Random variable wrt movement of agent/target
        Q      % process noise for platform (accel noise)
        
        % history and plotting options
        hist
        plot
        
    end
    
    methods
        
        function o = Dynamics( CONTROL, CLOCK, id )
            o = Default(o, CONTROL, CLOCK, id );
        end
        
    end
    
end

