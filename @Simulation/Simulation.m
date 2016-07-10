classdef Simulation < handle
    properties   ( SetAccess = public, GetAccess = public )
        
        mode            % single information sink (SDFC), multiple information sinks (MDFC)
        bSim            % binary value for terminate simulation (terminate when 0)
        
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
        function o = Simulation( nAgent, nSeed, bCentral, bPlot, bComm, mode )
            o = Default(o, nAgent, nSeed, bCentral, bPlot, bComm, mode );
        end
        
        o = get( o, varargin );
    end
    
end



