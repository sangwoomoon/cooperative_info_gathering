classdef Dynamics < handle
    %DYNAMICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        s       % current state [b_e, b_n, e,edot,n,ndot]
        bKFs    % binary array of state for using KF process.
    end
    
    methods
        function o = Dynamics( SIMULATION, CLOCK )
            o = Default(o, SIMULATION, CLOCK );
        end
        
        function o = TakeJacobian( o )
            
        end
        
        o = get( o, varargin );
    end
    
end

