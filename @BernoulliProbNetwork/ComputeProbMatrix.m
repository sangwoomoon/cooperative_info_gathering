function obj = ComputeProbMatrix( obj )
%COMPUTENETWORKGRAPH function determines the network graph between agents
%   it is now based on the distance between agents

    for iSender = 1 : length(obj.prob(:,1))
        for iReceiver = 1 : length(obj.prob(1,:))
            if iSender == iReceiver
                obj.prob(iSender,iReceiver) = 0; % not to communicate itself
            else
                % model given by the [Stachura and Frew(2017), Journal of
                % Field Robotics]
                X = (distance(obj.Z{iSender,iReceiver}.pos,obj.Z{iSender,iReceiver}.pos) - 461)/195;
                obj.prob(iSender,iReceiver) = 0.5*erfc(X)-0.085;
            end
        end
    end
    
    obj.hist.prob(end+1,:,:) = obj.prob; % store graph data

end


function dist = distance(a,b)
    dist = sqrt((a(1)-b(1))^2+(a(2)-b(2))^2);
end

