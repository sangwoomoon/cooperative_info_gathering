classdef DubinsDynamics < Dynamics
    %DubinsDynamics is a sub-class of Dynamics class
    
    properties

        % Platform motion model: platform states are [e,edot,n,ndot]
        F      % State transition matrix
        Gammma    % Process noise input matrix
        
        Gu      % Control input matrix
        
        v      % Random variable wrt movement of agent
        Q      % process noise for platform (accel noise)
    end
    
    methods
        function o = DubinsDynamics( SIMULATION, CLOCK )
            o@Dynamics(SIMULATION, CLOCK );
        end
        
        o = get( o, varargin );
    end
    
end

