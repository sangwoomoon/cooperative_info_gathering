classdef Network < handle
    %NETWORK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        spec % characteristics of network model
        
        topology % graph from network topology
        topograph % network graph defined by topology
        
        graph % binary matix (sender - receiver, could be asymmetric/symmetric and determined by topograph)
       
        prob % 100% - 0% of communication (could be usable for noise/binary)
        range % criteria of communication between agents (distance-based)
        
        Z % received data package from agents
        
        hist % history for prob matrix and network graph
        plot % plotting options
    end
    
    methods
        function obj = Network()
            obj = Declare(obj);
        end
        
        % make network graph : overloaded by characteristics of communicaiton model
        ComputeNetworkGraph(obj);
        
        % make probability matrix for communicaiton : overloaded by
        % characteristics of communicaiton model
        ComputeProbMatrix(obj);
        
        % receive package from an agent
        ReceivePackage(obj,RcvPackage);
        
        % send package to agent which ID is given by the input parameter
        SentPackage = SendPackage(obj,RcvAgentID);
        
        % basic togograph setting from topology input
        SetTopology(obj, nAgent, topology, Node_agents); % node is for star (otherwise, Node_agents is inputted as [])
    end
    
end

