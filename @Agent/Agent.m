classdef Agent < handle
    properties % ( SetAccess = public, GetAccess = public )
        
        id      % Agent id (integers)
        
        s       % current state [e,n]
        
        act     % candidate state (for action)
        actIdx  % action index
        
        util    % final utility (act -> R)
        
        utilCnd % utility candidate
        
        bDFC    % binary value for data fusion center
        
        rMove
        nMove
        
        alpha
        beta
        
        eta
        gamma
        
        w       % weight after computing utility
        
        COMM    % Communication Class (sub-class of agent)
                
        hist    % History
        plot    % Plot handle
        
    end % Properties
    
    methods
        function o = Agent( ENVIRONMENT, SIMULATION, CLOCK ,iAgent, iDFC )
             o = Default(o, ENVIRONMENT, SIMULATION, CLOCK ,iAgent, iDFC );
        end
        
        o = get( o, varargin );
    end
    
end



