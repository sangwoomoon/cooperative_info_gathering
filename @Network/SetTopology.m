function obj = SetTopology( obj, nAgent, topology, bNode )
%SETTOPOLOGY set a network graph from the topology input
% : star, ring, mesh, tree

switch topology
    
    case 'Star'
        obj.topology = 'star';
        
        obj.topograph = zeros(nAgent);
        for iAgent = 1 : nAgent
            if bNode(iAgent)
                obj.topograph(:,iAgent) = 1;
                obj.topograph(iAgent,iAgent) = 0;
            end
        end
        
    case 'Ring'
        obj.topology = 'ring';
        
        obj.topograph = zeros(nAgent);
        for iAgent = 1 : nAgent-1
            obj.topograph(iAgent,iAgent+1) = 1; % with respect to mid of ring
        end
        obj.topograph(nAgent,1) = 1; % with respect to end of ring (to make ring struct)
        
    case 'Mesh'
        obj.topology = 'mesh';
        
        obj.topograph = ones(nAgent);
        for iAgent = 1 : nAgent
            obj.topograph(iAgent,iAgent) = 0; 
        end
        
    case 'Tree'
        obj.topology = 'tree';
        
        obj.topograph = zeros(nAgent);
        for iAgent = 1 : nAgent-1
            obj.topograph(iAgent,iAgent+1) = 1;
        end
        
        for iAgent = 2 : nAgent
            obj.topograph(iAgent,iAgent-1) = 1;
        end
        
end


end

