classdef Agent < handle
    properties % ( SetAccess = public, GetAccess = public )
        
        id      % Agent id (integers)
        
        speed   % constant speed
        
        s       % current state [e,n,theta]
        
        bKFs    % binary array of state for using KF process.

        
        % Platform motion model: platform states are [e,n,theta]
        Fp      % State transition matrix
        Gamp    % Process noise input matrix
        
        Gu      % Control input matrix
        
        vp      % Random variable wrt movement of agent
        Qp      % process noise for platform (accel noise)
        
        TA      % Task Allocation Class (sub-class of agent)
        COMM    % Communication Class (sub-class of agent)
        MEASURE % Measurement Class (usb-class of agent)
        CONTROL % Control Class (sub-class of agent)
        
        FDDF    % Factorized DDF Class
       
        LOCAL_KF % KF Estimation Class (sub-class of agent)
        DECEN_KF % KF Estimation Class (sub-class of agent)
        FDDF_KF % Factorized DDF based KF Estimation Class 
                
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



