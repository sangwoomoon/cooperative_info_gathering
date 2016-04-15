classdef Simulation 
    properties   ( SetAccess = public, GetAccess = public )
        
        nTarget         % Number of Targets (pure target + landmark)
        nAgent          % Number of Agents
        nLandMark       % Number of Landmarks (considered as targets)
        
        iTarget         % operated target ID
        iAgent          % operated agent ID
        
        sRandom         % Specification of Random Variables (seeds for Random #)
        
        CENTRAL_KF      % Centralized KF Simulation
        
        VORONOI         % voronoi class handle
        LLOYD           % lloyd class handle
        
        plot            % plot for whole result (legends)
        
    end % Properties
    

    methods
        function o = Simulation( nAgent, nTarget )
            o = Default(o, nAgent, nTarget );
        end
        
        o = get( o, varargin );
    end
    
end



