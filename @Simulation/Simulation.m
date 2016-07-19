classdef Simulation 
    properties   ( SetAccess = public, GetAccess = public )
        
        nTarget         % Number of Targets
        nAgent          % Number of Agents
        nLandmark       % Number of Landmarks (treated as TARGET classes)
        
        iTarget         % operated target ID
        iAgent          % operated agent ID
        
        sRandom         % Specification of Random Variables (seeds for Random #)
        
        plot            % plot for whole result (legends)
        
    end % Properties
    

    methods
        function o = Simulation( nAgent, nTarget, nLandmark )
            o = Declare(o, nAgent, nTarget, nLandmark );
        end
        
        o = get( o, varargin );
    end
    
end



