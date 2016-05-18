classdef Dynamics < handle
    %DYNAMICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        s       % current state [e,edot,n,ndot]
        
        bKFs    % binary array of state for using KF process.

        
        % Platform motion model: platform states are [e,edot,n,ndot]
        Fp      % State transition matrix
        Gamp    % Process noise input matrix
        
        Gu      % Control input matrix
        
        vp      % Random variable wrt movement of agent
        Qp      % process noise for platform (accel noise)
    end
    
    methods
        function o = Dynamics( SIMULATION, CLOCK )
            o = Default(o, SIMULATION, CLOCK );
        end
        
        o = get( o, varargin );
    end
    
end

