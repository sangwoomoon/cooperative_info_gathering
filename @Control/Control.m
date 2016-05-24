classdef Control < handle
    properties % ( SetAccess = public, GetAccess = public )
        
        % symbolic form
        
        u_sym   % symbolic form of inputs
        
        % numeric form
        
        u       % acceleration input [eddot,nddot]
        
        hist    % History
        plot    % Plot handle
        
    end % Properties
    
    methods
        function o = Control( AGENT, TARGET, ENVIRONMENT )
             o = Default(o, AGENT, TARGET, ENVIRONMENT );
        end
        
        o = get( o, varargin );
    end
    
end



