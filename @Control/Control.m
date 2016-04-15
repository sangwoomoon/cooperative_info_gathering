classdef Control < handle
    properties % ( SetAccess = public, GetAccess = public )
        
        u       % acceleration input [eddot,nddot]
        
        kp      % proportional gain
        
        c       % centroid for Lloyd's algorithm
        
        hist    % History
        plot    % Plot handle
        
    end % Properties
    
    methods
        function o = Control( CLOCK )
             o = Default(o, CLOCK );
        end
        
        o = get( o, varargin );
    end
    
end



