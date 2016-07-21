function Z = SendPackage( obj, AgentID )
%SENDPACKAGE packs data to send an agent according to the network graph
%   Data flow : NETWORK --> Z

for iSender = 1 : length(obj.Z)
   if obj.graph(iSender,AgentID) == 1 % if connected
       Z(iSender).id = obj.Z(iSender).id; % received data from Network Class
       Z(iSender).meas = obj.Z(iSender).meas;
       Z(iSender).u = obj.Z(iSender).u;
       % Z(iSender).Phat = obj.Z(iSender).Phat;
       % Z(iSender).Xhat = obj.Z(iSender).Xhat;
   else
       Z(iSender).id = obj.Z(iSender).id; % received data from Network Class
       Z(iSender).meas = [];
       Z(iSender).u = [];
       % Z(iSender).Phat = [];
       % Z(iSender).Xhat = [];
   end
end


end

