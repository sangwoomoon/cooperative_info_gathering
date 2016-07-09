classdef Simulation < handle
    properties   ( SetAccess = public, GetAccess = public )
        
        nAgent          % Number of Agents
        
        iAgent          % operated agent ID
        
        sRandom         % Specification of Random Variables (seeds for Random #)
        
        bPlot           % binary value for plot
        bCentral        % binary value for centralization (central optimization is performed when 1)
        bComm           % binary value for communication-awareness
        
        DM              % central Decision Making handle
        
        plot            % plot for whole result
        hist            % history for global utility value
        
    end % Properties
    

    methods
        function o = Simulation( nAgent, nSeed, bCentral, bPlot, bComm )
            o = Default(o, nAgent, nSeed, bCentral, bPlot, bComm );
        end
        
        o = get( o, varargin );
    end
    
end



