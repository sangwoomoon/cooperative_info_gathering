function o = SendData( o, NETWORK, AGENT, SIMULATION )
%SENDDATA sends data to Network class
%   single agent -> network
%   SHOULD DISCUSS MAKING VECTOR FORM OF PACKAGE (NOT YET IMPLEMENTED!)

    id = AGENT.id; % allocate agent's id (for indexing)
    
    NETWORK.Z(id).id = id; % flag agent number 
    
    % add measurement data
    for iTarget = 1 : SIMULATION.nTarget
        NETWORK.Z(id).y{iTarget} = AGENT.MEASURE(iTarget).y;
    end
    
    % add input data
    NETWORK.Z(id).u = AGENT.CONTROL.u;

    % add estimates
    NETWORK.Z(id).Phat = AGENT.FDDF_KF.Phat; % beware of that it is combined by bias states
    NETWORK.Z(id).Xhat = AGENT.FDDF_KF.Xhat; % beware of that without bias (ignore 5,6th element)

end

