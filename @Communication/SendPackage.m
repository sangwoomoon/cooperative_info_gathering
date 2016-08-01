function NET = SendPackage( obj, Z_agent, NET )
%SENDPACKAGE receives Z from an agent and store this in the Network
%class
%   data flow : AGENT --> NETWORK
    
if isempty(NET.Z) % for the first receiving from agent
    NET.Z = Z_agent;
else
    NET.Z = [NET.Z; Z_agent];
end

end

