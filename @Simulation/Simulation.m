classdef Simulation < handle
    properties   ( SetAccess = public, GetAccess = public )
        
        nAgent          % Number of Agents
        
        iAgent          % operated agent ID
        
        sRandom         % Specification of Random Variables (seeds for Random #)
        
        bPlot           % binary value for plot
        
        DM              % central Decision Making handle
        
        plot            % plot for whole result
        hist            % history for global utility value
        
    end % Properties
    

    methods
        function o = Simulation( nAgent, nSeed, bPlot )
            o = Default(o, nAgent, nSeed, bPlot );
        end
        
        o = get( o, varargin );
    end
    
end



