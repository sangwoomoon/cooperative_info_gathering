classdef Target < handle
    properties ( SetAccess = public, GetAccess = public )
        id      % Target id (integers)
        
        x       % current state of target [e,e_dot,n,n_dot];
        
        bKFx    % binary array of state for using KF process.
        bLandMark % binary whether this target uses a landmark.
          
        % Target motion parameters (random walk)
        Ft      % State transition matrix
        
        Gt      % Process noise input matrix
        Gu      % target input matrix
        
        Qt      % target state process noise variances (velocity noise)
        
        vt      % Random variable wrt movement of target
        
        velMax  % maximum velocity (2 by 1)
        
        hist    % History
        plot    % Plot handle
        
    end % Properties
    
    methods
        function o = Target( CLOCK, ENVIRONMENT, iTarget, sRandom )
            o = Default( o, CLOCK, ENVIRONMENT, iTarget, sRandom);
        end
        
        o = get( o, varargin );
    end
    
end



