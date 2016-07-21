function obj = ReceivePackage( obj, Z )
%RECEIVEPACKAGE receives Z from an agent and store this in the Network
%class
%   data flow : AGENT --> NETWORK
    
if isempty(obj.Z) % for the first receiving from agent
    obj.Z = Z;
else
    obj.Z(Z.id) = Z;
end

end

