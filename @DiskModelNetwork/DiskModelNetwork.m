classdef DiskModelNetwork < Network
    %DISKMODELNETWORK is the sub-class of Network that considers disk model
    %for communication (prob = 1 within disk which radius is range, 0
    %outside disk)
    
    properties
        
    end
    
    methods
        function obj = DiskModelNetwork()
            obj@Network();
        end
        
        % make network graph : overloaded by characteristics of communicaiton model
        ComputeNetworkGraph(obj);
        
        % make probability matrix for communicaiton : overloaded by
        % characteristics of communicaiton model
        ComputeProbMatrix(obj);
        
    end
    
end

