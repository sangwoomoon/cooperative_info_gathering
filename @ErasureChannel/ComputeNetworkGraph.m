function obj = ComputeNetworkGraph( obj )
%COMPUTENETWORKGRAPH function determines the network graph between agents
%   it is now based on the distance between agents

    for iSender = 1 : length(obj.graph(:,1))
        for iReceiver = 1 : length(obj.graph(1,:))
            
            % take Bernoulli Distribution
            R = binornd(1,obj.prob(iSender,iReceiver));
            
            if obj.topograph(iSender,iReceiver) == 1 && R == 1 % determined by binary prob
                obj.graph(iSender,iReceiver) = 1; % they can communicate each other
            else
                obj.graph(iSender,iReceiver) = 0; % cannot communicate
            end            
        end
    end
    
    obj.hist.graph(end+1,:,:) = obj.graph; % store graph data

end

