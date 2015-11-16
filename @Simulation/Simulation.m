classdef Simulation 
    properties   ( SetAccess = public, GetAccess = public )
        
        nTarget         % Number of Targets
        nAgent          % Number of Agents
        
        iTarget         % operated target ID
        iAgent          % operated agent ID
        
        sRandom         % Specification of Random Variables (seeds for Random #)
        
        CENTRAL_KF      % Centralized KF Simulation
        
        plot            % plot for whole result (legends)
        
    end % Properties
    

    methods
        function o = Simulation( nAgent, nTarget )
            o = Default(o, nAgent, nTarget );
        end
        
        o = get( o, varargin );
    end
    
end



