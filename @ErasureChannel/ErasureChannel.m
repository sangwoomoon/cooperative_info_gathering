classdef ErasureChannel < Network
    %ERASURECHANNEL is the sub-class of Network that considers Packet
    %Erasure Channel model
    %for communication (prob: beta = sig(range,strength)
    
    properties
        
    end
    
    methods
        function obj = ErasureChannel()
            obj@Network();
        end
        
        % make network graph : overloaded by characteristics of communicaiton model
        ComputeNetworkGraph(obj);
        
        % make probability matrix for communicaiton : overloaded by
        % characteristics of communicaiton model
        ComputeProbMatrix(obj);
        
    end
    
end

