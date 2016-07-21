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
        
        % group jacobian : since this function should uses the AGENT, TARGET,
        % LANDMARK classes for centralized estimation.
        % should be modified because AGENT cannot know actual dynamics of TARGET
        jacobian = GroupJacobian(obj, AGENT, TARGET, CLOCK, option) % option: state / noise
    end
    
end



