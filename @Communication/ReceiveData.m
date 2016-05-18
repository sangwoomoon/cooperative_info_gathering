function o = ReceiveData( o, NETWORK, AGENT, SIMULATION )
%RECEIVEDATA receives data from Network class
%   network -> single agent
%   SHOULD DISCUSS VECTOR FORM OF PACKAGE (NOT YET IMPLEMENTED!)

for iSender = 1 : SIMULATION.nAgent
    if NETWORK.graph(iSender,AGENT.id) == 1 % when graph shows that two agents are connected
        o.Z(iSender).id = NETWORK.Z(iSender).id; % received data from Network Class
        o.Z(iSender).y = NETWORK.Z(iSender).y;
        o.Z(iSender).u = NETWORK.Z(iSender).u;
        o.Z(iSender).Phat = NETWORK.Z(iSender).Phat;
        o.Z(iSender).Xhat = NETWORK.Z(iSender).Xhat;
    else
        o.Z(iSender).id = [];
        o.Z(iSender).y = [];
        o.Z(iSender).u = [];
        o.Z(iSender).Phat = [];
        o.Z(iSender).Xhat = [];
    end
end


end

