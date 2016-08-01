classdef Simulation 
    properties   ( SetAccess = public, GetAccess = public )
        
        nTarget         % Number of Targets
        nAgent          % Number of Agents
        nLandmark       % Number of Landmarks (treated as TARGET classes)
        
        iTarget         % operated target ID
        iAgent          % operated agent ID
        
        iFigure         % index of figure that is handled now
        
        sRandom         % Specification of Random Variables (seeds for Random #)
        
        ESTIMATOR       % centralized estimation class
        NETWORK         % network class
        
        plot            % plot for whole result (legends)
        
    end % Properties
    

    methods
        function obj = Simulation( nAgent, nTarget, nLandmark, EstimationOption, NetworkOption )
            obj = Declare(obj, nAgent, nTarget, nLandmark, EstimationOption, NetworkOption );
        end
        
    end
    
end



