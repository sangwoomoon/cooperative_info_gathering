function Z = SendPackage( obj, AGENT )
%SENDDATA packs data and sends to Network class
%   single agent -> data package for communication

    Z.id = AGENT.id; % agent's id (for indexing)
    
    % add position of agent
    Z.pos = AGENT.DYNAMICS.GetPosition();
    
    % add measurement data
    Z.meas = AGENT.SENSOR.meas;
    
    % add input data
    Z.u = AGENT.CONTROL.u;

    % add estimates
    % NETWORK.Z(id).Phat = AGENT.FDDF_KF.Phat; % beware of that it is combined by bias states
    % NETWORK.Z(id).Xhat = AGENT.FDDF_KF.Xhat; % beware of that without bias (ignore 5,6th element)

end

