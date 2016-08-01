function obj = DeliverPackage( obj )
%DELIVERPACKAGE makes the package array with respect to the network graph

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

end

