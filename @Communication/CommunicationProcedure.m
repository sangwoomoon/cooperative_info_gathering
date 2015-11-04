function o = CommunicationProcedure (o, AGENT, SIMULATION, id)

iComm = 1;
for iSender = 1 : SIMULATION.nAgent
    if o.C(iSender,id) == 1 % Receiver and Sender should be different
        Z(iComm).id = iSender;
        Z(iComm).y = AGENT(iSender).MEASURE.y;
        Z(iComm).u = AGENT(iSender).CONTROL.u;
        Z(iComm).Rp = AGENT(iSender).MEASURE.Rp;
        Z(iComm).Rt = AGENT(iSender).MEASURE.Rt;
    else % Receiver and Sender should be different
        Z(iComm).id = [];
        Z(iComm).y = [];
        Z(iComm).u = [];
        Z(iComm).Rp = [];
        Z(iComm).Rt = [];
    end
    iComm = iComm + 1;
end

o.Z = Z; % allocate data package to agent

end