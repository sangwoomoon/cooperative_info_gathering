classdef Target < handle
    properties ( SetAccess = public, GetAccess = public )
        id      % Target id (integers)
        
        x       % current state of target [e,e_dot,n,n_dot];
          
        % Target motion parameters (random walk)
        Ft      % State transition matrix
        
        Gt      % Process noise input matrix
        Gu      % target input matrix
        
        Qt      % target state process noise variances (velocity noise)
        
        vt      % Random variable wrt movement of target
        
        hist    % History
        plot    % Plot handle
        
    end % Properties
    
    methods
        function o = Target( Simulation, Clock, iTarget )
            o = Default( o, Simulation, Clock , iTarget);
        end
        
        o = get( o, varargin );
    end
    
end



