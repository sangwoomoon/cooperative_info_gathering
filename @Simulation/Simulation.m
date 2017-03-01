classdef Simulation < handle
    properties   ( SetAccess = public, GetAccess = public )
        
        nSim            % simulation running number
        
        cost            % simulation performance cost (array, 1: centralized, 2: Fusion_MMNB, 3: Fusion_diag)
        
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
        function obj = Simulation( nSim, nAgent, nTarget, nLandmark )
            obj = Declare(obj, nSim, nAgent, nTarget, nLandmark );
        end
        
    end
    
end



