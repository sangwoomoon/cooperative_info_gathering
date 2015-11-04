classdef Agent < handle
    properties % ( SetAccess = public, GetAccess = public )
        
        id      % Agent id (integers)
        
        s       % current state [e,edot,n,ndot]
        
        % Platform motion model: platform states are [e,edot,n,ndot]
        Fp      % State transition matrix
        Gamp    % Process noise input matrix
        
        Gu      % Control input matrix
        
        vp      % Random variable wrt movement of agent
        Qp      % process noise for platform (accel noise)
        
        TA      % Task Allocation Class (sub-class of agent)
        COMM    % Communication Class (sub-class of agent)
        MEASURE % Measurement Class (usb-class of agent)
        CONTROL % Control Class (sub-class of agent)
        LOCAL_KF % KF Estimation Class (sub-class of agent)
        DECEN_KF % KF Estimation Class (sub-class of agent)
        
        hist    % History
        plot    % Plot handle
        
    end % Properties
    
    methods
        function o = Agent( TARGET, ENVIRONMENT, SIMULATION, CLOCK ,iAgent )
             o = Default(o, TARGET, ENVIRONMENT, SIMULATION, CLOCK ,iAgent );
        end
        
        o = get( o, varargin );
    end
    
end



