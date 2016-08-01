classdef SigmoidModelNetwork < Network
    %SIGMOIDMODELNETWORK is the sub-class of Network that considers sigmoid function model
    %for communication (prob: beta = sig(range,strength)
    
    properties
        
    end
    
    methods
        function obj = SigmoidModelNetwork()
            obj@Network();
        end
        
        % make network graph : overloaded by characteristics of communicaiton model
        ComputeNetworkGraph(obj);
        
        % make probability matrix for communicaiton : overloaded by
        % characteristics of communicaiton model
        ComputeProbMatrix(obj);
        
    end
    
end

