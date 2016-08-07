function obj = DeliverPackage( obj, AGENT )
%DELIVERPACKAGE makes the package array with respect to the network graph

% nullify package inside NETWORK class
obj.NullifyPackage();

%--- Make Package and send this to the Network Class (not physical class!) for each agent ----
for iAgent = 1 : length(AGENT)
    Z = AGENT(iAgent).FUSION.CreatePackage(AGENT(iAgent).id, AGENT(iAgent).DYNAMICS.GetPosition(),...
        AGENT(iAgent).SENSOR.bTrack, AGENT(iAgent).SENSOR.meas, AGENT(iAgent).CONTROL.u,...
        AGENT(iAgent).ESTIMATOR.Phat, AGENT(iAgent).ESTIMATOR.xhat);
    
    obj = AGENT(iAgent).COMM.SendPackage(Z,obj);
end

obj.ComputeProbMatrix();
obj.ComputeNetworkGraph();

for iSender = 1 : length(obj.Z(:,1))
    for iReceiver = 1 : length(obj.Z(1,:))
        % nullify package when iSender is not connected to iReceiver
        if obj.graph(iSender,iReceiver) == 0
            obj.Z{iSender,iReceiver} = [];
        end
    end
end

%--- Receive Data from Network Class ----
for iAgent = 1 : length(AGENT)
    AGENT(iAgent).COMM.ReceivePackage(obj.Z(:,iAgent)');
end



end

