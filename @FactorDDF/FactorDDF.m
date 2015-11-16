classdef FactorDDF < handle
    properties % ( SetAccess = public, GetAccess = public )
        
        XhatDDF % Xhat processed by DDF
        PhatDDF % Phat processed by DDF
        
        XhatMgn % Marginalized KF state (extract target state from Xhat)
        PhatMgn % Marginalized Covariance Matrix of KF process
        
        omega   % factorizing parameter
        
        M       % Information matrix
        m       % Information vector
        
        hist    % History
        plot    % Plot handle
        
    end % Properties
    
    methods
        function o = FactorDDF( AGENT, SIMULATION )
             o = Default(o, AGENT, SIMULATION );
        end
        
        o = get( o, varargin );
    end
    
end



