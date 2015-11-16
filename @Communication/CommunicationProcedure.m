function o = CommunicationProcedure (o, AGENT, SIMULATION, id)

iComm = 1;
for iSender = 1 : SIMULATION.nAgent
    if o.C(iSender,id) == 1 % Receiver and Sender should be different
        Z(iComm).id = iSender;
        Z(iComm).y = AGENT(iSender).MEASURE.y;
        Z(iComm).u = AGENT(iSender).CONTROL.u;
        % Z(iComm).R = AGENT(iSender).DECEN_KF.R; % beware of that this is from FDDF_KF
        Z(iComm).Phat = AGENT(iSender).FDDF_KF.Phat; % beware of that it is combined by bias states
        Z(iComm).Xhat = AGENT(iSender).FDDF_KF.Xhat; % beware of that without bias (ignore 5,6th element)
    else % Receiver and Sender should be different
        Z(iComm).id = [];
        Z(iComm).y = [];
        Z(iComm).u = [];
        % Z(iComm).R = [];
        Z(iComm).Phat = [];
        Z(iComm).Xhat = [];
    end
    iComm = iComm + 1;
end

o.Z = Z; % allocate data package to agent

end