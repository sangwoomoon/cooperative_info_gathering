function o = CommunicationProcedure (o, AGENT, SIMULATION )

% dataset : agent id / status, input, and measured target id/measurement

for iSender = 1 : SIMULATION.nAgent
    if o.C(iSender) == 1 
        Z(iSender).agentID = iSender;
        Z(iSender).targetID = AGENT(iSender).TA.TrackID;
        Z(iSender).s = AGENT(iSender).s;
        Z(iSender).y = AGENT(iSender).MEASURE.y;
        Z(iSender).u = AGENT(iSender).CONTROL.u;
    else 
        Z(iSender).agentID = [];
        Z(iSender).targetID = [];
        Z(iSender).s = [];
        Z(iSender).y = [];
        Z(iSender).u = [];
    end
end

o.Z = Z; % allocate data package to agent

end